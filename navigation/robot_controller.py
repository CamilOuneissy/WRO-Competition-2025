# navigation/robot_controller.py

import cv2
import numpy as np
import time
import logging
import threading
from enum import Enum
from typing import Optional, Tuple, List

class RobotState(Enum):
    IDLE = "idle"
    OPEN_CHALLENGE = "open_challenge"
    OBSTACLE_CHALLENGE = "obstacle_challenge"
    PARKING = "parking"
    EMERGENCY_STOP = "emergency_stop"
    COMPLETED = "completed"

class Direction(Enum):
    CLOCKWISE = "clockwise"
    COUNTER_CLOCKWISE = "counter_clockwise"

class RobotController:
    """Main robot controller for WRO Future Engineers 2025"""
    
    def __init__(self, config, motor_controller, left_sensor, right_sensor, color_detector, gpio_controller):
        self.config = config
        self.motor = motor_controller
        self.left_sensor = left_sensor
        self.right_sensor = right_sensor
        self.color_detector = color_detector
        self.gpio = gpio_controller
        
        # State management
        self.state = RobotState.IDLE
        self.direction = Direction.CLOCKWISE
        self.laps_completed = 0
        self.target_laps = 3
        
        # Navigation parameters with safe defaults
        self.base_speed = config.get('motor', 'default_speed') or 0.4
        self.corner_speed = config.get('navigation', 'corner_speed') or 0.3
        self.obstacle_speed = config.get('navigation', 'obstacle_speed') or 0.25
        self.distance_threshold = config.get('navigation', 'distance_threshold') or 20
        self.turn_timeout = config.get('navigation', 'turn_timeout') or 3.0
        
        # Camera setup
        self.camera = None
        self.camera_active = False
        self.frame = None
        self.camera_thread = None
        
        # Challenge tracking
        self.start_time = None
        self.challenge_active = False
        self.parking_attempted = False
        
        # Traffic sign tracking
        self.last_red_sign_time = 0
        self.last_green_sign_time = 0
        self.sign_cooldown = 2.0  # Seconds between sign detections
        
        logging.info("Robot controller initialized")
    
    def initialize_camera(self):
        """Initialize camera for obstacle detection"""
        try:
            camera_index = self.config.get('camera', 'camera_index') or 0
            self.camera = cv2.VideoCapture(camera_index)
            
            if not self.camera.isOpened():
                logging.warning("Failed to open camera - continuing without camera")
                return False
            
            # Set camera properties
            width = self.config.get('camera', 'frame_width') or 640
            height = self.config.get('camera', 'frame_height') or 480
            fps = self.config.get('camera', 'camera_fps') or 30
            
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.camera.set(cv2.CAP_PROP_FPS, fps)
            
            logging.info(f"Camera initialized: {width}x{height} @ {fps}fps")
            return True
            
        except Exception as e:
            logging.warning(f"Camera initialization failed: {e} - continuing without camera")
            return False
    
    def start_camera_thread(self):
        """Start camera capture thread"""
        if self.camera is None:
            if not self.initialize_camera():
                return False
        
        self.camera_active = True
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()
        return True
    
    def stop_camera_thread(self):
        """Stop camera capture thread"""
        self.camera_active = False
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=1.0)
        
        if self.camera:
            self.camera.release()
            self.camera = None
    
    def _camera_loop(self):
        """Camera capture loop running in separate thread"""
        while self.camera_active and self.camera:
            try:
                ret, frame = self.camera.read()
                if ret:
                    self.frame = frame
                else:
                    logging.warning("Failed to capture frame")
                    time.sleep(0.1)
            except Exception as e:
                logging.error(f"Camera loop error: {e}")
                time.sleep(0.1)
    
    def get_current_frame(self):
        """Get the latest camera frame"""
        return self.frame.copy() if self.frame is not None else None
    
    def start_open_challenge(self, direction=Direction.CLOCKWISE):
        """Start open challenge (no obstacles)"""
        logging.info(f"Starting open challenge - Direction: {direction.value}")
        
        self.state = RobotState.OPEN_CHALLENGE
        self.direction = direction
        self.laps_completed = 0
        self.start_time = time.time()
        self.challenge_active = True
        
        # Start sensors
        self.left_sensor.start_continuous_reading()
        self.right_sensor.start_continuous_reading()
        
        # Main challenge loop
        try:
            while self.challenge_active and self.laps_completed < self.target_laps:
                self._open_challenge_step()
                time.sleep(0.05)  # 20Hz control loop
                
        except Exception as e:
            logging.error(f"Open challenge error: {e}")
            self.emergency_stop()
        finally:
            self._challenge_cleanup()
    
    def start_obstacle_challenge(self, direction=Direction.CLOCKWISE):
        """Start obstacle challenge (with traffic signs)"""
        logging.info(f"Starting obstacle challenge - Direction: {direction.value}")
        
        self.state = RobotState.OBSTACLE_CHALLENGE
        self.direction = direction
        self.laps_completed = 0
        self.start_time = time.time()
        self.challenge_active = True
        self.parking_attempted = False
        
        # Start sensors and camera
        self.left_sensor.start_continuous_reading()
        self.right_sensor.start_continuous_reading()
        
        if not self.start_camera_thread():
            logging.error("Failed to start camera for obstacle challenge")
            return False
        
        # Main challenge loop
        try:
            while self.challenge_active:
                if self.laps_completed < self.target_laps:
                    self._obstacle_challenge_step()
                elif not self.parking_attempted:
                    self._parking_phase()
                else:
                    break
                    
                time.sleep(0.05)  # 20Hz control loop
                
        except Exception as e:
            logging.error(f"Obstacle challenge error: {e}")
            self.emergency_stop()
        finally:
            self._challenge_cleanup()
    
    def _open_challenge_step(self):
        """Single step of open challenge logic"""
        # Get sensor readings
        left_distance = self.left_sensor.get_latest_reading()
        right_distance = self.right_sensor.get_latest_reading()
        
        if left_distance is None or right_distance is None:
            logging.warning("Invalid sensor readings")
            return
        
        # Wall following logic
        steering, speed = self._calculate_wall_following(left_distance, right_distance)
        
        # Apply controls
        self.motor.set_speed(speed)
        self.motor.set_steering(steering)
        
        # Check for lap completion (simple heuristic)
        self._check_lap_completion()
    
    def _obstacle_challenge_step(self):
        """Single step of obstacle challenge logic"""
        # Get sensor readings
        left_distance = self.left_sensor.get_latest_reading()
        right_distance = self.right_sensor.get_latest_reading()
        
        if left_distance is None or right_distance is None:
            logging.warning("Invalid sensor readings")
            return
        
        # Get camera frame and detect traffic signs
        frame = self.get_current_frame()
        red_sign, green_sign = None, None
        
        if frame is not None:
            red_sign, green_sign = self._detect_traffic_signs(frame)
        
        # Calculate navigation based on obstacles and walls
        steering, speed = self._calculate_obstacle_navigation(
            left_distance, right_distance, red_sign, green_sign
        )
        
        # Apply controls
        self.motor.set_speed(speed)
        self.motor.set_steering(steering)
        
        # Check for lap completion
        self._check_lap_completion()
    
    def _detect_traffic_signs(self, frame):
        """Detect red and green traffic signs"""
        if frame is None:
            return None, None
        
        # Convert to HSV for color detection
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detect red and green signs
        red_mask, red_bbox = self.color_detector.detect_color(hsv_frame, 'red')
        green_mask, green_bbox = self.color_detector.detect_color(hsv_frame, 'green')
        
        current_time = time.time()
        
        # Filter out rapid repeated detections
        red_sign = None
        green_sign = None
        
        if red_bbox and (current_time - self.last_red_sign_time) > self.sign_cooldown:
            red_sign = red_bbox
            self.last_red_sign_time = current_time
            logging.info(f"Red sign detected: {red_bbox}")
        
        if green_bbox and (current_time - self.last_green_sign_time) > self.sign_cooldown:
            green_sign = green_bbox
            self.last_green_sign_time = current_time
            logging.info(f"Green sign detected: {green_bbox}")
        
        return red_sign, green_sign
    
    def _calculate_wall_following(self, left_distance, right_distance):
        """Calculate steering and speed for wall following"""
        # Target distance from wall
        target_distance = self.distance_threshold
        
        # Use the closer wall for following
        if left_distance < right_distance:
            # Follow left wall
            error = left_distance - target_distance
            steering = -error * 0.02  # Proportional control
        else:
            # Follow right wall
            error = right_distance - target_distance
            steering = error * 0.02
        
        # Limit steering
        steering = max(-1.0, min(1.0, steering))
        
        # Adjust speed based on proximity to walls
        min_distance = min(left_distance, right_distance)
        if min_distance < 10:
            speed = self.obstacle_speed
        elif min_distance < 15:
            speed = self.corner_speed
        else:
            speed = self.base_speed
        
        return steering, speed
    
    def _calculate_obstacle_navigation(self, left_distance, right_distance, red_sign, green_sign):
        """Calculate navigation considering traffic signs"""
        # Start with wall following
        steering, speed = self._calculate_wall_following(left_distance, right_distance)
        
        # Adjust for traffic signs
        if red_sign:
            # Red sign: keep to the right
            x1, y1, x2, y2 = red_sign
            sign_center_x = (x1 + x2) / 2
            frame_center = 320  # Assuming 640px width
            
            if sign_center_x < frame_center:
                # Sign is on the left, steer right
                steering += 0.3
            else:
                # Sign is on the right, steer more right to pass on right side
                steering += 0.1
                
        elif green_sign:
            # Green sign: keep to the left
            x1, y1, x2, y2 = green_sign
            sign_center_x = (x1 + x2) / 2
            frame_center = 320
            
            if sign_center_x > frame_center:
                # Sign is on the right, steer left
                steering -= 0.3
            else:
                # Sign is on the left, steer more left to pass on left side
                steering -= 0.1
        
        # Reduce speed when approaching signs
        if red_sign or green_sign:
            speed = min(speed, self.obstacle_speed)
        
        # Limit steering
        steering = max(-1.0, min(1.0, steering))
        
        return steering, speed
    
    def _parking_phase(self):
        """Handle parking after completing 3 laps"""
        logging.info("Starting parking phase")
        self.state = RobotState.PARKING
        self.parking_attempted = True
        
        # Simple parking strategy: look for magenta markers
        frame = self.get_current_frame()
        if frame is not None:
            # Look for magenta parking markers
            parking_spot = self._detect_parking_spot(frame)
            if parking_spot:
                self._execute_parking(parking_spot)
            else:
                # Continue searching for parking spot
                self.motor.set_speed(0.2)
                self.motor.set_steering(0.1)
        
        time.sleep(2.0)  # Give time for parking maneuver
        self._complete_challenge()
    
    def _detect_parking_spot(self, frame):
        """Detect magenta parking markers"""
        # Convert to HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Magenta color range (approximate)
        lower_magenta = np.array([140, 100, 100])
        upper_magenta = np.array([170, 255, 255])
        
        mask = cv2.inRange(hsv_frame, lower_magenta, upper_magenta)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour (parking markers)
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:
                return cv2.boundingRect(largest_contour)
        
        return None
    
    def _execute_parking(self, parking_spot):
        """Execute parking maneuver"""
        logging.info(f"Executing parking at: {parking_spot}")
        
        # Simple parking: align and move into spot
        x, y, w, h = parking_spot
        frame_center = 320
        spot_center = x + w/2
        
        # Align with parking spot
        if abs(spot_center - frame_center) > 50:
            if spot_center < frame_center:
                self.motor.set_steering(-0.3)
            else:
                self.motor.set_steering(0.3)
            self.motor.set_speed(0.2)
            time.sleep(1.0)
        
        # Move into parking spot
        self.motor.set_steering(0)
        self.motor.set_speed(0.3)
        time.sleep(2.0)
        
        # Stop
        self.motor.stop()
        logging.info("Parking completed")
    
    def _check_lap_completion(self):
        """Check if a lap has been completed"""
        # Simple time-based lap detection (placeholder)
        if self.start_time:
            elapsed = time.time() - self.start_time
            expected_lap_time = 30  # Seconds per lap (estimate)
            
            expected_laps = int(elapsed / expected_lap_time)
            if expected_laps > self.laps_completed:
                self.laps_completed = expected_laps
                logging.info(f"Lap {self.laps_completed} completed")
    
    def _complete_challenge(self):
        """Complete the current challenge"""
        self.state = RobotState.COMPLETED
        self.challenge_active = False
        self.motor.stop()
        
        elapsed_time = time.time() - self.start_time if self.start_time else 0
        logging.info(f"Challenge completed in {elapsed_time:.1f} seconds")
        logging.info(f"Laps completed: {self.laps_completed}")
    
    def emergency_stop(self):
        """Emergency stop procedure"""
        logging.warning("EMERGENCY STOP ACTIVATED")
        self.state = RobotState.EMERGENCY_STOP
        self.challenge_active = False
        self.motor.emergency_stop()
    
    def _challenge_cleanup(self):
        """Cleanup after challenge completion"""
        self.motor.stop()
        self.left_sensor.stop_continuous_reading()
        self.right_sensor.stop_continuous_reading()
        self.stop_camera_thread()
        
        logging.info("Challenge cleanup completed")
    
    def get_status(self):
        """Get current robot status"""
        return {
            'state': self.state.value,
            'direction': self.direction.value,
            'laps_completed': self.laps_completed,
            'challenge_active': self.challenge_active,
            'motor_status': self.motor.get_status(),
            'left_distance': self.left_sensor.get_latest_reading(),
            'right_distance': self.right_sensor.get_latest_reading()
        }
    
    def test_systems(self):
        """Test all robot systems"""
        logging.info("Starting system tests...")
        
        # Test motors
        logging.info("Testing motors...")
        self.motor.test_motor(duration=1.0)
        
        # Test sensors
        logging.info("Testing sensors...")
        self.left_sensor.test_sensor(duration=3.0)
        self.right_sensor.test_sensor(duration=3.0)
        
        # Test camera
        logging.info("Testing camera...")
        if self.initialize_camera():
            ret, frame = self.camera.read()
            if ret:
                logging.info(f"Camera test successful: {frame.shape}")
            else:
                logging.error("Camera test failed")
            self.camera.release()
        
        logging.info("System tests completed")
    
    def cleanup(self):
        """Cleanup all resources"""
        logging.info("Cleaning up robot controller...")
        self.emergency_stop()
        self._challenge_cleanup()
        self.motor.cleanup()
        
        if hasattr(self.left_sensor, 'cleanup'):
            self.left_sensor.cleanup()
        if hasattr(self.right_sensor, 'cleanup'):
            self.right_sensor.cleanup()
        
        logging.info("Robot controller cleanup completed")