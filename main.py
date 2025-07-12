#!/usr/bin/env python3
# main.py

import os
import sys
import time
import signal
import logging
import threading
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from config.robot_config import RobotConfig
from hardware.gpio_controller import GPIOController
from hardware.motor_controller import MotorController
from hardware.ultrasonic_sensor import UltrasonicSensor
from vision.color_detector import EnhancedColorDetector
from navigation.robot_controller import RobotController, Direction

class WROFutureEngineersRobot:
    """Main WRO Future Engineers Robot Class"""
    
    def __init__(self, config_path="robot_config.json"):
        self.config = RobotConfig(config_path)
        self.gpio = None
        self.motor = None
        self.left_sensor = None
        self.right_sensor = None
        self.color_detector = None
        self.robot_controller = None
        
        # Control flags
        self.running = True
        self.challenge_mode = None  # 'open' or 'obstacle'
        self.direction = Direction.CLOCKWISE
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        logging.info("WRO Future Engineers Robot initialized")
    
    def initialize_hardware(self):
        """Initialize all hardware components"""
        try:
            logging.info("Initializing hardware components...")
            
            # Initialize GPIO controller
            logging.info("Initializing GPIO controller...")
            self.gpio = GPIOController()
            
            # Initialize motor controller
            logging.info("Initializing motor controller...")
            self.motor = MotorController(self.config, self.gpio)
            
            # Initialize ultrasonic sensors
            logging.info("Initializing ultrasonic sensors...")
            left_trig = self.config.get('gpio', 'left_ultrasonic_trig') or 4
            left_echo = self.config.get('gpio', 'left_ultrasonic_echo') or 17
            right_trig = self.config.get('gpio', 'right_ultrasonic_trig') or 27
            right_echo = self.config.get('gpio', 'right_ultrasonic_echo') or 22
            
            self.left_sensor = UltrasonicSensor(
                left_trig, left_echo, self.gpio, 
                name="left_sensor", max_distance=200
            )
            
            self.right_sensor = UltrasonicSensor(
                right_trig, right_echo, self.gpio, 
                name="right_sensor", max_distance=200
            )
            
            # Initialize color detector
            logging.info("Initializing color detector...")
            self.color_detector = EnhancedColorDetector(self.config)
            
            # Initialize robot controller
            logging.info("Initializing robot controller...")
            self.robot_controller = RobotController(
                self.config, self.motor, self.left_sensor, 
                self.right_sensor, self.color_detector, self.gpio
            )
            
            # Setup GPIO pins for buttons and LEDs
            logging.info("Setting up GPIO interface...")
            self._setup_gpio_interface()
            
            logging.info("Hardware initialization completed successfully")
            return True
            
        except Exception as e:
            logging.error(f"Hardware initialization failed: {e}")
            logging.error(f"Error type: {type(e)}")
            import traceback
            logging.error(f"Traceback: {traceback.format_exc()}")
            return False
    
    def _setup_gpio_interface(self):
        """Setup GPIO pins for user interface"""
        try:
            # Setup start button
            start_button_pin = self.config.get('gpio', 'start_button_pin') or 2
            if start_button_pin:
                self.gpio.setup_pin(start_button_pin, self.gpio.IN)
                logging.info(f"Start button configured on pin {start_button_pin}")
            
            # Setup LED indicator
            led_pin = self.config.get('gpio', 'led_pin') or 3
            if led_pin:
                self.gpio.setup_pin(led_pin, self.gpio.OUT, initial=self.gpio.LOW)
                logging.info(f"LED configured on pin {led_pin}")
            
            logging.info("GPIO interface setup completed")
            
        except Exception as e:
            logging.error(f"GPIO interface setup failed: {e}")
            # Don't fail initialization for GPIO interface issues
            pass
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        logging.info(f"Received signal {signum}, initiating shutdown...")
        self.running = False
    
    def wait_for_start_button(self):
        """Wait for start button press"""
        start_button_pin = self.config.get('gpio', 'start_button_pin') or 2
        led_pin = self.config.get('gpio', 'led_pin') or 3
        
        if not start_button_pin:
            logging.warning("No start button configured, using timeout")
            time.sleep(3)
            return True
        
        logging.info("Waiting for start button press...")
        
        # Blink LED to indicate ready state
        if led_pin:
            try:
                for _ in range(10):
                    if not self.running:
                        return False
                    self.gpio.output(led_pin, self.gpio.HIGH)
                    time.sleep(0.2)
                    self.gpio.output(led_pin, self.gpio.LOW)
                    time.sleep(0.2)
            except Exception as e:
                logging.warning(f"LED blinking failed: {e}")
        
        # Wait for button press
        start_time = time.time()
        timeout = 30.0  # 30 second timeout
        
        while self.running and (time.time() - start_time) < timeout:
            try:
                if self.gpio.input(start_button_pin) == self.gpio.HIGH:
                    logging.info("Start button pressed!")
                    
                    # Solid LED to indicate start
                    if led_pin:
                        try:
                            self.gpio.output(led_pin, self.gpio.HIGH)
                        except Exception as e:
                            logging.warning(f"LED control failed: {e}")
                    
                    # Debounce
                    time.sleep(0.5)
                    return True
            except Exception as e:
                logging.warning(f"Button reading failed: {e}")
                break
            
            time.sleep(0.1)
        
        logging.warning("Start button timeout or shutdown requested")
        return False
    
    def run_open_challenge(self):
        """Run the open challenge"""
        logging.info("Starting Open Challenge")
        
        try:
            if not self.wait_for_start_button():
                return False
            
            self.robot_controller.start_open_challenge(self.direction)
            
            # Wait for completion
            while (self.robot_controller.challenge_active and 
                   self.robot_controller.state.value != "completed" and 
                   self.running):
                time.sleep(0.5)
            
            logging.info("Open challenge completed")
            return True
            
        except Exception as e:
            logging.error(f"Open challenge error: {e}")
            return False
    
    def run_obstacle_challenge(self):
        """Run the obstacle challenge"""
        logging.info("Starting Obstacle Challenge")
        
        try:
            if not self.wait_for_start_button():
                return False
            
            self.robot_controller.start_obstacle_challenge(self.direction)
            
            # Wait for completion
            while (self.robot_controller.challenge_active and 
                   self.robot_controller.state.value != "completed" and 
                   self.running):
                time.sleep(0.5)
            
            logging.info("Obstacle challenge completed")
            return True
            
        except Exception as e:
            logging.error(f"Obstacle challenge error: {e}")
            return False
    
    def run_system_tests(self):
        """Run comprehensive system tests"""
        logging.info("Running system tests...")
        
        try:
            # Test individual components
            self.robot_controller.test_systems()
            
            # Test basic movements
            logging.info("Testing basic movements...")
            self.robot_controller.motor.set_speed(0.3)
            time.sleep(1.0)
            self.robot_controller.motor.set_steering(0.5)
            time.sleep(1.0)
            self.robot_controller.motor.set_steering(-0.5)
            time.sleep(1.0)
            self.robot_controller.motor.stop()
            
            logging.info("System tests completed successfully")
            return True
            
        except Exception as e:
            logging.error(f"System test failed: {e}")
            return False
    
    def interactive_mode(self):
        """Interactive mode for testing and configuration"""
        print("\n=== WRO Future Engineers Robot - Interactive Mode ===")
        print("1. Run System Tests")
        print("2. Open Challenge")
        print("3. Obstacle Challenge")
        print("4. Set Direction (Current: {})".format(self.direction.value))
        print("5. Show Status")
        print("6. Calibrate Colors")
        print("7. Exit")
        
        while self.running:
            try:
                choice = input("\nEnter choice (1-7): ").strip()
                
                if choice == "1":
                    self.run_system_tests()
                
                elif choice == "2":
                    self.run_open_challenge()
                
                elif choice == "3":
                    self.run_obstacle_challenge()
                
                elif choice == "4":
                    if self.direction == Direction.CLOCKWISE:
                        self.direction = Direction.COUNTER_CLOCKWISE
                    else:
                        self.direction = Direction.CLOCKWISE
                    print(f"Direction set to: {self.direction.value}")
                
                elif choice == "5":
                    status = self.robot_controller.get_status()
                    print(f"Robot Status: {status}")
                
                elif choice == "6":
                    print("Color calibration not implemented in interactive mode")
                    print("Use the configuration files to adjust color ranges")
                
                elif choice == "7":
                    break
                
                else:
                    print("Invalid choice. Please enter 1-7.")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                logging.error(f"Interactive mode error: {e}")
    
    def run_autonomous_mode(self, challenge_type):
        """Run in autonomous mode (no user interaction)"""
        logging.info(f"Running autonomous mode: {challenge_type}")
        
        # Wait a bit for startup
        time.sleep(2.0)
        
        if challenge_type == "open":
            return self.run_open_challenge()
        elif challenge_type == "obstacle":
            return self.run_obstacle_challenge()
        else:
            logging.error(f"Unknown challenge type: {challenge_type}")
            return False
    
    def cleanup(self):
        """Cleanup all resources"""
        logging.info("Cleaning up robot resources...")
        
        try:
            if self.robot_controller:
                self.robot_controller.cleanup()
            
            if self.motor:
                self.motor.cleanup()
            
            if self.gpio:
                self.gpio.cleanup()
            
            logging.info("Cleanup completed successfully")
            
        except Exception as e:
            logging.error(f"Cleanup error: {e}")

def main():
    """Main entry point"""
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='WRO Future Engineers Robot 2025')
    parser.add_argument('--config', default='robot_config.json', 
                       help='Configuration file path')
    parser.add_argument('--mode', choices=['interactive', 'auto'], default='interactive',
                       help='Operation mode')
    parser.add_argument('--challenge', choices=['open', 'obstacle'], 
                       help='Challenge type for auto mode')
    parser.add_argument('--direction', choices=['cw', 'ccw'], default='cw',
                       help='Driving direction (cw=clockwise, ccw=counter-clockwise)')
    parser.add_argument('--test', action='store_true',
                       help='Run system tests only')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug logging')
    
    args = parser.parse_args()
    
    # Setup logging level
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Create robot instance
    robot = WROFutureEngineersRobot(args.config)
    
    try:
        # Initialize hardware
        if not robot.initialize_hardware():
            logging.error("Failed to initialize hardware")
            return 1
        
        # Set direction
        if args.direction == 'ccw':
            robot.direction = Direction.COUNTER_CLOCKWISE
        
        # Run based on mode
        if args.test:
            robot.run_system_tests()
        elif args.mode == 'interactive':
            robot.interactive_mode()
        elif args.mode == 'auto':
            if not args.challenge:
                logging.error("Challenge type required for auto mode")
                return 1
            robot.run_autonomous_mode(args.challenge)
        
        return 0
        
    except Exception as e:
        logging.error(f"Main execution error: {e}")
        return 1
        
    finally:
        robot.cleanup()

if __name__ == "__main__":
    sys.exit(main())