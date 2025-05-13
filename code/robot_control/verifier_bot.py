#!/usr/bin/env python3
"""
BhoomiMitra - Smart Agriculture System
Verifier Bot Control Module

This script controls the Verifier Bot, which follows the Leader Bot
and verifies seed placement using computer vision.

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

# Camera settings
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# ArUco marker detection
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

# Communication settings
MESH_PORT = 5555
BROADCAST_IP = "192.168.4.255"  # For ESP-NOW or WiFi mesh network
ROBOT_ID = "VERIFIER_BOT"

# Field mapping and navigation
field_map = [
    {"id": 1, "x": 100, "y": 100, "verified": False},
    {"id": 2, "x": 200, "y": 100, "verified": False},
    {"id": 3, "x": 300, "y": 100, "verified": False},
    {"id": 4, "x": 100, "y": 200, "verified": False},
    {"id": 5, "x": 200, "y": 200, "verified": False},
    {"id": 6, "x": 300, "y": 200, "verified": False},
    {"id": 7, "x": 100, "y": 300, "verified": False},
    {"id": 8, "x": 200, "y": 300, "verified": False},
    {"id": 9, "x": 300, "y": 300, "verified": False}
]

# Robot state
current_position = {"x": 0, "y": 0, "orientation": 0}  # Orientation in degrees
is_verifying = False
target_point = None
leader_bot_position = None
recently_planted_points = []

# Initialize GPIO
def setup_gpio():
    """Initialize GPIO pins for motors and sensors."""
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

# Camera and vision functions
def setup_camera():
    """Initialize the camera."""
    global camera
    
    try:
        camera = cv2.VideoCapture(0)  # Use first camera
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        
        # Check if camera opened successfully
        if not camera.isOpened():
            print("Error: Could not open camera.")
            return False
        
        print("Camera initialized")
        return True
    
    except Exception as e:
        print(f"Error initializing camera: {e}")
        return False

def capture_image():
    """Capture an image from the camera."""
    if not camera.isOpened():
        print("Error: Camera not available.")
        return None
    
    # Capture frame
    ret, frame = camera.read()
    
    if not ret:
        print("Error: Could not read frame.")
        return None
    
    return frame

def detect_aruco_markers(frame):
    """Detect ArUco markers in the image."""
    if frame is None:
        return None
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect markers
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
    
    # If markers detected
    if ids is not None:
        # Draw markers on frame for debugging
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Process each marker
        markers = []
        for i in range(len(ids)):
            # Calculate marker center
            c = corners[i][0]
            center = (int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4),
                      int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4))
            
            # Add to markers list
            markers.append({
                "id": ids[i][0],
                "center": center,
                "corners": corners[i][0].tolist()
            })
        
        return markers, frame
    
    return None, frame

def detect_seed(frame, point_id):
    """Detect if a seed is present in the image."""
    if frame is None:
        return False
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply threshold to highlight seeds (adjust threshold values as needed)
    _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)
    
    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter contours by size to identify seeds
    seed_detected = False
    for contour in contours:
        area = cv2.contourArea(contour)
        if 10 < area < 200:  # Adjust area range based on seed size
            # Draw contour for debugging
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            seed_detected = True
    
    # Save image for debugging
    timestamp = int(time.time())
    filename = f"seed_detection_point_{point_id}_{timestamp}.jpg"
    cv2.imwrite(filename, frame)
    
    return seed_detected

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

# Verification function
def verify_seed_at_point(point):
    """Verify if a seed was properly planted at the specified point."""
    global is_verifying
    
    is_verifying = True
    print(f"Verifying seed at point {point['id']}")
    
    # Navigate to the point
    if navigate_to_point(point):
        # Stop and stabilize
        stop()
        time.sleep(1)
        
        # Capture image
        frame = capture_image()
        
        # Detect seed
        seed_detected = detect_seed(frame, point["id"])
        
        if seed_detected:
            print(f"Seed verified at point {point['id']}")
            point["verified"] = True
            
            # Notify other robots
            broadcast_message({
                "type": "VERIFICATION_SUCCESS",
                "point_id": point["id"],
                "position": {"x": point["x"], "y": point["y"]},
                "robot_id": ROBOT_ID,
                "timestamp": time.time()
            })
        else:
            print(f"No seed detected at point {point['id']}!")
            
            # Notify other robots
            broadcast_message({
                "type": "VERIFICATION_FAILED",
                "point_id": point["id"],
                "position": {"x": point["x"], "y": point["y"]},
                "robot_id": ROBOT_ID,
                "timestamp": time.time()
            })
    
    is_verifying = False
    return point["verified"]

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
    global leader_bot_position, recently_planted_points
    
    while True:
        try:
            data, addr = udp_socket.recvfrom(1024)
            message = json.loads(data.decode())
            
            # Process message
            if message["robot_id"] != ROBOT_ID:  # Ignore own messages
                print(f"Received from {message['robot_id']}: {message['type']}")
                
                if message["type"] == "SEED_PLANTED":
                    # Leader bot planted a seed
                    point_id = message["point_id"]
                    position = message["position"]
                    
                    print(f"Leader bot planted seed at point {point_id}.")
                    
                    # Add to recently planted points queue
                    recently_planted_points.append({
                        "id": point_id,
                        "x": position["x"],
                        "y": position["y"],
                        "timestamp": message["timestamp"]
                    })
                
                elif message["robot_id"] == "LEADER_BOT" and "position" in message:
                    # Update leader bot position
                    leader_bot_position = message["position"]
        
        except Exception as e:
            print(f"Error receiving message: {e}")

# Main control loop
def main():
    """Main function for the Verifier Bot."""
    global recently_planted_points
    
    try:
        # Setup
        setup_gpio()
        if not setup_camera():
            print("Error: Failed to initialize camera. Exiting.")
            return
        
        setup_communication()
        
        print("Verifier Bot initialized. Starting verification mission...")
        time.sleep(2)
        
        # Main loop - verify seeds planted by the Leader Bot
        while True:
            # Check if there are any recently planted points to verify
            if recently_planted_points:
                # Sort by timestamp (oldest first)
                recently_planted_points.sort(key=lambda x: x["timestamp"])
                
                # Get the oldest point
                point = recently_planted_points.pop(0)
                
                # Find the corresponding point in our field map
                for map_point in field_map:
                    if map_point["id"] == point["id"]:
                        # Verify this point
                        verify_seed_at_point(map_point)
                        break
                
                # Small delay between verifications
                time.sleep(2)
            else:
                # No points to verify, wait for new ones
                print("Waiting for new points to verify...")
                time.sleep(5)
                
                # Check if all points are verified
                all_verified = True
                for point in field_map:
                    if not point["verified"]:
                        all_verified = False
                        break
                
                if all_verified:
                    print("All points verified! Mission complete.")
                    break
        
        # Mission complete
        print("Verification mission completed.")
        
    except KeyboardInterrupt:
        print("Program stopped by user")
    
    finally:
        # Cleanup
        stop()
        if 'camera' in globals() and camera is not None:
            camera.release()
        GPIO.cleanup()
        udp_socket.close()
        print("Cleanup complete")

if __name__ == "__main__":
    main()
