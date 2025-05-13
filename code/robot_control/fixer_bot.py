#!/usr/bin/env python3
"""
BhoomiMitra - Smart Agriculture System
Fixer Bot Control Module

This script controls the Fixer Bot, which responds to verification failures
and re-plants seeds at locations where the Verifier Bot detected issues.

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
ROBOT_ID = "FIXER_BOT"

# Field mapping and navigation
field_map = [
    {"id": 1, "x": 100, "y": 100, "fixed": False},
    {"id": 2, "x": 200, "y": 100, "fixed": False},
    {"id": 3, "x": 300, "y": 100, "fixed": False},
    {"id": 4, "x": 100, "y": 200, "fixed": False},
    {"id": 5, "x": 200, "y": 200, "fixed": False},
    {"id": 6, "x": 300, "y": 200, "fixed": False},
    {"id": 7, "x": 100, "y": 300, "fixed": False},
    {"id": 8, "x": 200, "y": 300, "fixed": False},
    {"id": 9, "x": 300, "y": 300, "fixed": False}
]

# Robot state
current_position = {"x": 0, "y": 0, "orientation": 0}  # Orientation in degrees
seed_count = 100  # Initial seed count
is_fixing = False
target_point = None
failed_points = []  # Points that need fixing

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

# Fixing function
def fix_seed_at_point(point):
    """Fix a seed placement at the specified point."""
    global is_fixing
    
    is_fixing = True
    print(f"Fixing seed at point {point['id']}")
    
    # Navigate to the point
    if navigate_to_point(point):
        # Stop and stabilize
        stop()
        time.sleep(1)
        
        # Prepare the soil (optional - could add soil preparation mechanism)
        print("Preparing soil...")
        time.sleep(1)
        
        # Dispense seed
        if dispense_seed():
            # Mark as fixed
            point["fixed"] = True
            
            # Notify other robots
            broadcast_message({
                "type": "SEED_FIXED",
                "point_id": point["id"],
                "position": {"x": point["x"], "y": point["y"]},
                "robot_id": ROBOT_ID,
                "timestamp": time.time()
            })
            
            print(f"Successfully fixed seed at point {point['id']}")
            time.sleep(1)  # Wait for seed to settle
        else:
            print(f"Failed to fix seed at point {point['id']} - out of seeds!")
    
    is_fixing = False
    return point["fixed"]

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
    global failed_points
    
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
                    position = message["position"]
                    
                    print(f"Verification failed for point {point_id}. Adding to fix queue.")
                    
                    # Add to failed points queue
                    failed_points.append({
                        "id": point_id,
                        "x": position["x"],
                        "y": position["y"],
                        "timestamp": message["timestamp"]
                    })
                    
                    # Find the point in our field map and mark as not fixed
                    for point in field_map:
                        if point["id"] == point_id:
                            point["fixed"] = False
                            break
                
                elif message["type"] == "VERIFICATION_SUCCESS":
                    # Confirmation that a seed was successfully planted
                    point_id = message["point_id"]
                    
                    # Check if this point is in our failed points queue
                    for i, point in enumerate(failed_points):
                        if point["id"] == point_id:
                            # Remove from queue
                            failed_points.pop(i)
                            print(f"Point {point_id} was verified successfully, removing from fix queue.")
                            break
        
        except Exception as e:
            print(f"Error receiving message: {e}")

# Main control loop
def main():
    """Main function for the Fixer Bot."""
    global failed_points
    
    try:
        # Setup
        setup_gpio()
        setup_communication()
        
        print("Fixer Bot initialized. Starting fixing mission...")
        time.sleep(2)
        
        # Main loop - fix seeds that failed verification
        while True:
            # Check if there are any failed points to fix
            if failed_points:
                # Sort by timestamp (oldest first)
                failed_points.sort(key=lambda x: x["timestamp"])
                
                # Get the oldest point
                point = failed_points[0]  # Don't remove yet, wait for verification
                
                # Find the corresponding point in our field map
                for map_point in field_map:
                    if map_point["id"] == point["id"] and not map_point["fixed"]:
                        # Fix this point
                        if fix_seed_at_point(map_point):
                            print(f"Successfully fixed point {point['id']}. Waiting for verification.")
                        else:
                            print(f"Failed to fix point {point['id']}.")
                        break
                
                # Small delay between fixes
                time.sleep(2)
            else:
                # No points to fix, wait for failures
                print("Waiting for verification failures...")
                time.sleep(5)
                
                # Check if all points are fixed
                all_fixed = True
                for point in field_map:
                    if not point["fixed"]:
                        all_fixed = False
                        break
                
                if all_fixed and not failed_points:
                    print("All points fixed! Mission complete.")
                    break
        
        # Mission complete
        print(f"Fixing mission completed. Used {100 - seed_count} seeds for fixes.")
        
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
