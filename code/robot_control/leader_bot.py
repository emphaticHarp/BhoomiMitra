#!/usr/bin/env python3
"""
BhoomiMitra - Smart Agriculture System
Leader Bot Control Module

This script controls the Leader Bot, which is responsible for planting seeds
at predefined GPS/mapped points in the field.

Author: Soumyajyoti Banik
License: MIT
"""

import time
import math
import cv2
import numpy as np
import RPi.GPIO as GPIO
from gpiozero import Motor
import socket
import json
import threading

# Motor control pins (using L298N driver)
LEFT_MOTOR_FORWARD = 17
LEFT_MOTOR_BACKWARD = 18
RIGHT_MOTOR_FORWARD = 22
RIGHT_MOTOR_BACKWARD = 23

# Ultrasonic sensor pins
TRIG_FRONT = 24
ECHO_FRONT = 25
TRIG_REAR = 27
ECHO_REAR = 4

# Seed dispenser servo pin
SEED_SERVO = 12

# ArUco marker detection
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

# Communication settings
MESH_PORT = 5555
BROADCAST_IP = "192.168.4.255"  # For ESP-NOW or WiFi mesh network
ROBOT_ID = "LEADER_BOT"

# Field mapping and navigation
field_map = [
    {"id": 1, "x": 100, "y": 100, "planted": False},
    {"id": 2, "x": 200, "y": 100, "planted": False},
    {"id": 3, "x": 300, "y": 100, "planted": False},
    {"id": 4, "x": 100, "y": 200, "planted": False},
    {"id": 5, "x": 200, "y": 200, "planted": False},
    {"id": 6, "x": 300, "y": 200, "planted": False},
    {"id": 7, "x": 100, "y": 300, "planted": False},
    {"id": 8, "x": 200, "y": 300, "planted": False},
    {"id": 9, "x": 300, "y": 300, "planted": False}
]

# Robot state
current_position = {"x": 0, "y": 0, "orientation": 0}  # Orientation in degrees
seed_count = 100  # Initial seed count
is_planting = False
target_point = None

# Initialize GPIO
def setup_gpio():
    """Initialize GPIO pins for motors, sensors, and servo."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Setup motor pins
    GPIO.setup(LEFT_MOTOR_FORWARD, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_BACKWARD, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_FORWARD, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_BACKWARD, GPIO.OUT)
    
    # Setup ultrasonic sensor pins
    GPIO.setup(TRIG_FRONT, GPIO.OUT)
    GPIO.setup(ECHO_FRONT, GPIO.IN)
    GPIO.setup(TRIG_REAR, GPIO.OUT)
    GPIO.setup(ECHO_REAR, GPIO.IN)
    
    # Setup seed dispenser servo
    GPIO.setup(SEED_SERVO, GPIO.OUT)
    global seed_servo
    seed_servo = GPIO.PWM(SEED_SERVO, 50)  # 50Hz frequency
    seed_servo.start(7.5)  # Middle position
    
    print("GPIO initialized")

# Motor control functions
def move_forward(speed=100):
    """Move robot forward at specified speed (0-100)."""
    speed_value = speed / 100.0  # Convert to 0-1 range
    
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)
    
    print(f"Moving forward at speed {speed}%")

def move_backward(speed=100):
    """Move robot backward at specified speed (0-100)."""
    speed_value = speed / 100.0  # Convert to 0-1 range
    
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.HIGH)
    
    print(f"Moving backward at speed {speed}%")

def turn_left(speed=70):
    """Turn robot left at specified speed (0-100)."""
    speed_value = speed / 100.0  # Convert to 0-1 range
    
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)
    
    print(f"Turning left at speed {speed}%")

def turn_right(speed=70):
    """Turn robot right at specified speed (0-100)."""
    speed_value = speed / 100.0  # Convert to 0-1 range
    
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.HIGH)
    
    print(f"Turning right at speed {speed}%")

def stop():
    """Stop all motors."""
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)
    
    print("Stopped")

# Seed dispenser control
def dispense_seed():
    """Activate seed dispenser to plant one seed."""
    global seed_count
    
    if seed_count <= 0:
        print("Error: Out of seeds!")
        return False
    
    print("Dispensing seed...")
    
    # Move servo to dispense position
    seed_servo.ChangeDutyCycle(2.5)  # 0 degrees
    time.sleep(1)
    
    # Return to closed position
    seed_servo.ChangeDutyCycle(7.5)  # 90 degrees
    time.sleep(0.5)
    
    # Decrement seed count
    seed_count -= 1
    print(f"Seed dispensed. Remaining seeds: {seed_count}")
    
    return True

# Ultrasonic sensor functions
def measure_distance(trigger, echo):
    """Measure distance using ultrasonic sensor."""
    # Send trigger pulse
    GPIO.output(trigger, GPIO.LOW)
    time.sleep(0.00001)
    GPIO.output(trigger, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trigger, GPIO.LOW)
    
    # Wait for echo to start
    pulse_start = time.time()
    timeout = pulse_start + 0.04  # 40ms timeout
    
    while GPIO.input(echo) == 0 and time.time() < timeout:
        pulse_start = time.time()
    
    # Wait for echo to end
    pulse_end = time.time()
    timeout = pulse_end + 0.04  # 40ms timeout
    
    while GPIO.input(echo) == 1 and time.time() < timeout:
        pulse_end = time.time()
    
    # Calculate distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound = 343 m/s = 34300 cm/s
    distance = round(distance, 2)
    
    return distance

def check_obstacles():
    """Check for obstacles using ultrasonic sensors."""
    front_distance = measure_distance(TRIG_FRONT, ECHO_FRONT)
    rear_distance = measure_distance(TRIG_REAR, ECHO_REAR)
    
    print(f"Front distance: {front_distance} cm, Rear distance: {rear_distance} cm")
    
    # Return True if obstacle detected
    return front_distance < 20  # Obstacle within 20cm

# Navigation functions
def calculate_heading(target):
    """Calculate heading to target point."""
    dx = target["x"] - current_position["x"]
    dy = target["y"] - current_position["y"]
    
    # Calculate angle in degrees
    target_angle = math.degrees(math.atan2(dy, dx))
    
    # Normalize to 0-360 range
    if target_angle < 0:
        target_angle += 360
    
    return target_angle

def calculate_distance(target):
    """Calculate distance to target point."""
    dx = target["x"] - current_position["x"]
    dy = target["y"] - current_position["y"]
    
    return math.sqrt(dx*dx + dy*dy)

def navigate_to_point(target):
    """Navigate to a specific point on the field."""
    global current_position, target_point
    
    target_point = target
    print(f"Navigating to point {target['id']} at ({target['x']}, {target['y']})")
    
    while calculate_distance(target) > 10:  # Within 10cm is close enough
        # Check for obstacles
        if check_obstacles():
            print("Obstacle detected! Stopping.")
            stop()
            time.sleep(1)
            continue
        
        # Calculate heading to target
        target_heading = calculate_heading(target)
        heading_diff = target_heading - current_position["orientation"]
        
        # Normalize heading difference to -180 to 180
        if heading_diff > 180:
            heading_diff -= 360
        elif heading_diff < -180:
            heading_diff += 360
        
        # Adjust orientation if needed
        if abs(heading_diff) > 10:
            if heading_diff > 0:
                turn_right()
                time.sleep(0.1)
                current_position["orientation"] = (current_position["orientation"] + 5) % 360
            else:
                turn_left()
                time.sleep(0.1)
                current_position["orientation"] = (current_position["orientation"] - 5) % 360
            stop()
        else:
            # Move forward
            move_forward()
            time.sleep(0.2)
            stop()
            
            # Update position (simplified - in real implementation, use encoders or GPS)
            distance_moved = 5  # Assume we moved 5cm
            current_position["x"] += distance_moved * math.cos(math.radians(current_position["orientation"]))
            current_position["y"] += distance_moved * math.sin(math.radians(current_position["orientation"]))
    
    print(f"Reached point {target['id']}")
    return True

# Planting function
def plant_seed_at_point(point):
    """Plant a seed at the specified point."""
    global is_planting
    
    is_planting = True
    print(f"Planting seed at point {point['id']}")
    
    # Navigate to the point
    if navigate_to_point(point):
        # Stop and stabilize
        stop()
        time.sleep(1)
        
        # Dispense seed
        if dispense_seed():
            # Mark as planted
            point["planted"] = True
            
            # Notify other robots
            broadcast_message({
                "type": "SEED_PLANTED",
                "point_id": point["id"],
                "position": {"x": point["x"], "y": point["y"]},
                "robot_id": ROBOT_ID,
                "timestamp": time.time()
            })
            
            print(f"Successfully planted seed at point {point['id']}")
            time.sleep(1)  # Wait for seed to settle
    
    is_planting = False
    return point["planted"]

# Communication functions
def setup_communication():
    """Setup UDP socket for robot communication."""
    global udp_socket
    
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    # Bind to receive messages
    udp_socket.bind(("0.0.0.0", MESH_PORT))
    
    # Start listening thread
    threading.Thread(target=listen_for_messages, daemon=True).start()
    
    print("Communication initialized")

def broadcast_message(message):
    """Broadcast a message to other robots."""
    try:
        message_json = json.dumps(message)
        udp_socket.sendto(message_json.encode(), (BROADCAST_IP, MESH_PORT))
        print(f"Broadcasted: {message['type']}")
    except Exception as e:
        print(f"Error broadcasting message: {e}")

def listen_for_messages():
    """Listen for incoming messages from other robots."""
    while True:
        try:
            data, addr = udp_socket.recvfrom(1024)
            message = json.loads(data.decode())
            
            # Process message
            if message["robot_id"] != ROBOT_ID:  # Ignore own messages
                print(f"Received from {message['robot_id']}: {message['type']}")
                
                if message["type"] == "VERIFICATION_FAILED":
                    # Verifier bot found an issue with a planted seed
                    point_id = message["point_id"]
                    print(f"Verification failed for point {point_id}. Will retry planting.")
                    
                    # Find the point and mark as not planted
                    for point in field_map:
                        if point["id"] == point_id:
                            point["planted"] = False
                            break
                
                elif message["type"] == "VERIFICATION_SUCCESS":
                    # Confirmation that a seed was successfully planted
                    point_id = message["point_id"]
                    print(f"Verification successful for point {point_id}.")
        
        except Exception as e:
            print(f"Error receiving message: {e}")

# Main control loop
def main():
    """Main function for the Leader Bot."""
    try:
        # Setup
        setup_gpio()
        setup_communication()
        
        print("Leader Bot initialized. Starting seeding mission...")
        time.sleep(2)
        
        # Main loop - plant seeds at each point in the field map
        while True:
            # Find next unplanted point
            next_point = None
            for point in field_map:
                if not point["planted"]:
                    next_point = point
                    break
            
            if next_point:
                # Plant seed at this point
                plant_seed_at_point(next_point)
                
                # Notify progress
                planted_count = sum(1 for point in field_map if point["planted"])
                print(f"Progress: {planted_count}/{len(field_map)} points planted")
                
                # Small delay between points
                time.sleep(2)
            else:
                # All points planted
                print("All points planted! Mission complete.")
                break
        
        # Mission complete
        print(f"Seeding mission completed. Used {100 - seed_count} seeds.")
        
    except KeyboardInterrupt:
        print("Program stopped by user")
    
    finally:
        # Cleanup
        stop()
        seed_servo.stop()
        GPIO.cleanup()
        udp_socket.close()
        print("Cleanup complete")

if __name__ == "__main__":
    main()
