# hardware/motor_controller.py

import logging
import time
import threading

class MotorController:
    """Enhanced motor controller with safety features and smooth control"""
    
    def __init__(self, config, gpio_controller):
        self.config = config
        self.gpio = gpio_controller
        
        # Motor state
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.target_speed = 0.0
        self.target_steering = 0.0
        
        # Safety features
        self.is_emergency_stop = False
        self.max_acceleration = config.get('motor', 'acceleration_rate', 0.1) or 0.1
        self.last_update_time = time.time()
        
        # Motor control thread
        self.control_thread = None
        self.control_running = False
        
        # GPIO pin assignments with safe defaults
        self.enable_pin = config.get('gpio', 'motor_enable_pin') or 18
        self.dir_pin1 = config.get('gpio', 'motor_direction_pin1') or 19
        self.dir_pin2 = config.get('gpio', 'motor_direction_pin2') or 20
        self.servo_pin = config.get('gpio', 'servo_pin') or 21
        
        # Motor parameters with safe defaults
        self.max_speed = config.get('motor', 'max_speed') or 0.8
        self.min_speed = config.get('motor', 'min_speed') or 0.1
        
        # FIXED: Add safe defaults for steering parameters
        self.steering_center = config.get('motor', 'steering_center') or 7.5
        self.steering_range = config.get('motor', 'steering_range') or 2.5
        
        # Debug logging to see what we got
        logging.info(f"Motor config: steering_center={self.steering_center}, steering_range={self.steering_range}")
        logging.info(f"Motor config: max_speed={self.max_speed}, min_speed={self.min_speed}")
        
        # Validate that we have valid values
        if self.steering_center is None:
            logging.warning("steering_center is None, using default 7.5")
            self.steering_center = 7.5
            
        if self.steering_range is None:
            logging.warning("steering_range is None, using default 2.5")
            self.steering_range = 2.5
        
        # Ensure they are numeric
        try:
            self.steering_center = float(self.steering_center)
            self.steering_range = float(self.steering_range)
        except (ValueError, TypeError):
            logging.error(f"Invalid steering values: center={self.steering_center}, range={self.steering_range}")
            self.steering_center = 7.5
            self.steering_range = 2.5
        
        logging.info(f"Final steering config: center={self.steering_center}, range={self.steering_range}")
        
        self.initialize_hardware()
        logging.info("Motor controller initialized")
    
    def initialize_hardware(self):
        """Initialize GPIO pins and PWM"""
        try:
            # Setup motor control pins
            self.gpio.setup_pin(self.enable_pin, self.gpio.OUT)
            self.gpio.setup_pin(self.dir_pin1, self.gpio.OUT)
            self.gpio.setup_pin(self.dir_pin2, self.gpio.OUT)
            self.gpio.setup_pin(self.servo_pin, self.gpio.OUT)
            
            # Initialize PWM
            self.motor_pwm = self.gpio.setup_pwm(self.enable_pin, 1000)  # 1kHz for motor
            self.servo_pwm = self.gpio.setup_pwm(self.servo_pin, 50)     # 50Hz for servo
            
            # Start PWM with safe values - FIXED to ensure steering_center is valid
            self.motor_pwm.start(0)  # Motor stopped
            
            # Double-check steering_center before using it
            if self.steering_center is None or not isinstance(self.steering_center, (int, float)):
                logging.error(f"Invalid steering_center value: {self.steering_center}, using 7.5")
                self.steering_center = 7.5
            
            logging.info(f"Starting servo PWM with steering_center: {self.steering_center}")
            self.servo_pwm.start(self.steering_center)  # Servo centered
            
            # Set initial direction (stopped)
            self.gpio.output(self.dir_pin1, self.gpio.LOW)
            self.gpio.output(self.dir_pin2, self.gpio.LOW)
            
            # Start control thread for smooth operation
            self.start_control_thread()
            
        except Exception as e:
            logging.error(f"Motor hardware initialization failed: {e}")
            raise
    
    def start_control_thread(self):
        """Start the motor control thread for smooth operation"""
        self.control_running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        logging.debug("Motor control thread started")
    
    def stop_control_thread(self):
        """Stop the motor control thread"""
        self.control_running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
        logging.debug("Motor control thread stopped")
    
    def _control_loop(self):
        """Main control loop for smooth motor operation"""
        self.last_update_time = time.time()  # Initialize here
    
        while self.control_running:
            try:
                current_time = time.time()
                dt = current_time - self.last_update_time
            
                # Ensure dt is valid
                if dt <= 0 or dt > 1.0:
                    dt = 0.02  # Default 50Hz control rate
            
                self.last_update_time = current_time
            
                # Update motor speed with acceleration limiting
                self._update_speed(dt)
            
                # Update steering with smoothing
                self._update_steering(dt)
            
                # Sleep for control frequency (50Hz)
                time.sleep(0.02)
            
            except Exception as e:
                logging.error(f"Motor control loop error: {e}")
                time.sleep(0.1)
    
    def _update_speed(self, dt):
        """Update motor speed with smooth acceleration"""
        if self.is_emergency_stop:
            self.target_speed = 0
            self.current_speed = 0
            self.motor_pwm.ChangeDutyCycle(0)
            return
        
        # Calculate speed difference
        speed_diff = self.target_speed - self.current_speed
        max_change = (self.max_acceleration or 0.1) * dt
        
        # Limit acceleration
        if abs(speed_diff) > max_change:
            if speed_diff > 0:
                self.current_speed += max_change
            else:
                self.current_speed -= max_change
        else:
            self.current_speed = self.target_speed
        
        # Apply to hardware
        self._apply_motor_speed()
    
    def _update_steering(self, dt):
        """Update steering with smoothing"""
        # Simple smoothing for steering (could be enhanced)
        steering_diff = self.target_steering - self.current_steering
        max_steering_change = 2.0 * dt  # Max 2.0 units per second
        
        if abs(steering_diff) > max_steering_change:
            if steering_diff > 0:
                self.current_steering += max_steering_change
            else:
                self.current_steering -= max_steering_change
        else:
            self.current_steering = self.target_steering
        
        # Apply to hardware
        self._apply_steering()
    
    def _apply_motor_speed(self):
        """Apply current speed to motor hardware"""
        # Calculate duty cycle (0-100%)
        duty_cycle = abs(self.current_speed) * 100
        duty_cycle = max(0, min(100, duty_cycle))
        
        # Set PWM duty cycle
        self.motor_pwm.ChangeDutyCycle(duty_cycle)
        
        # Set direction
        if self.current_speed > 0:
            # Forward
            self.gpio.output(self.dir_pin1, self.gpio.HIGH)
            self.gpio.output(self.dir_pin2, self.gpio.LOW)
        elif self.current_speed < 0:
            # Reverse
            self.gpio.output(self.dir_pin1, self.gpio.LOW)
            self.gpio.output(self.dir_pin2, self.gpio.HIGH)
        else:
            # Stop
            self.gpio.output(self.dir_pin1, self.gpio.LOW)
            self.gpio.output(self.dir_pin2, self.gpio.LOW)
    
    def _apply_steering(self):
        """Apply current steering to servo hardware"""
        # Convert steering (-1.0 to 1.0) to PWM duty cycle
        duty_cycle = self.steering_center + (self.current_steering * self.steering_range)
        
        # Apply servo limits (typical servo range 5-10% duty cycle)
        duty_cycle = max(5.0, min(10.0, duty_cycle))
        
        # Set servo PWM
        self.servo_pwm.ChangeDutyCycle(duty_cycle)
    
    def set_speed(self, speed):
        """Set target motor speed (-1.0 to 1.0)"""
        # Validate and limit speed
        speed = max(-self.max_speed, min(self.max_speed, speed))
        
        # Apply minimum speed threshold
        if abs(speed) < self.min_speed and speed != 0:
            speed = self.min_speed if speed > 0 else -self.min_speed
        
        self.target_speed = speed
        logging.debug(f"Target speed set to {speed:.2f}")
    
    def set_steering(self, angle):
        """Set steering angle (-1.0 to 1.0, left negative, right positive)"""
        # Validate and limit steering
        angle = max(-1.0, min(1.0, angle))
        self.target_steering = angle
        logging.debug(f"Target steering set to {angle:.2f}")
    
    def emergency_stop(self):
        """Emergency stop - immediate motor shutdown"""
        self.is_emergency_stop = True
        self.target_speed = 0
        self.current_speed = 0
        self.motor_pwm.ChangeDutyCycle(0)
        self.gpio.output(self.dir_pin1, self.gpio.LOW)
        self.gpio.output(self.dir_pin2, self.gpio.LOW)
        logging.warning("EMERGENCY STOP ACTIVATED")
    
    def reset_emergency_stop(self):
        """Reset emergency stop condition"""
        self.is_emergency_stop = False
        logging.info("Emergency stop reset")
    
    def stop(self):
        """Graceful stop"""
        self.set_speed(0)
        self.set_steering(0)
        logging.info("Motor stopped")
    
    def get_status(self):
        """Get current motor status"""
        return {
            'current_speed': self.current_speed,
            'target_speed': self.target_speed,
            'current_steering': self.current_steering,
            'target_steering': self.target_steering,
            'emergency_stop': self.is_emergency_stop,
            'control_running': self.control_running
        }
    
    def test_motor(self, duration=2.0):
        """Test motor functionality"""
        logging.info("Starting motor test...")
        
        try:
            # Test forward
            logging.info("Testing forward motion...")
            self.set_speed(0.3)
            time.sleep(duration)
            
            # Test stop
            logging.info("Testing stop...")
            self.set_speed(0)
            time.sleep(1)
            
            # Test reverse
            logging.info("Testing reverse motion...")
            self.set_speed(-0.3)
            time.sleep(duration)
            
            # Test steering
            logging.info("Testing steering...")
            self.set_speed(0)
            self.set_steering(0.5)  # Right
            time.sleep(1)
            self.set_steering(-0.5)  # Left
            time.sleep(1)
            self.set_steering(0)  # Center
            
            logging.info("Motor test completed successfully")
            
        except Exception as e:
            logging.error(f"Motor test failed: {e}")
        finally:
            self.stop()
    
    def cleanup(self):
        """Cleanup motor resources"""
        logging.info("Cleaning up motor controller...")
        
        # Stop control thread
        self.stop_control_thread()
        
        # Stop motors
        self.emergency_stop()
        
        # Stop PWM
        try:
            if hasattr(self.motor_pwm, 'stop'):
                self.motor_pwm.stop()
            if hasattr(self.servo_pwm, 'stop'):
                self.servo_pwm.stop()
        except Exception as e:
            logging.error(f"Error stopping PWM: {e}")
        
        logging.info("Motor controller cleanup completed")