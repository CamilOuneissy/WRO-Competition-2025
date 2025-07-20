#!/usr/bin/env python3
"""
Simple GUI Test Suite for Obstacle Avoidance Robot
Click buttons to test individual components or run all tests
Enhanced with wall centering functionality
"""

import RPi.GPIO as GPIO
import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time

# GPIO Pin Configuration
LEFT_TRIG = 4
LEFT_ECHO = 17
RIGHT_TRIG = 27
RIGHT_ECHO = 22
ENA = 18
IN1 = 19
IN2 = 20
SERVO_PIN = 12

# Constants
SERVO_CENTER = 6.41
SERVO_LEFT = 8.81 #8.13     # Flipped: was 4.7
SERVO_RIGHT = 4.01 #4.7     # Flipped: was 8.13
DC_MOTOR_SPEED = 30
SAFE_DISTANCE = 150

class RobotTestGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Robot Test Suite")
        self.root.geometry("600x500")
        self.root.configure(bg='#f0f0f0')
        
        self.is_testing = False
        self.setup_gpio()
        self.create_widgets()
        
    def setup_gpio(self):
        """Initialize GPIO and PWM."""
        GPIO.setmode(GPIO.BCM)
        
        # Setup pins
        GPIO.setup(LEFT_TRIG, GPIO.OUT)
        GPIO.setup(LEFT_ECHO, GPIO.IN)
        GPIO.setup(RIGHT_TRIG, GPIO.OUT)
        GPIO.setup(RIGHT_ECHO, GPIO.IN)
        GPIO.setup(ENA, GPIO.OUT)
        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        
        # Initialize outputs
        GPIO.output(LEFT_TRIG, False)
        GPIO.output(RIGHT_TRIG, False)
        GPIO.output(IN1, False)
        GPIO.output(IN2, False)
        
        # PWM setup
        self.motor_pwm = GPIO.PWM(ENA, 1000)
        self.servo_pwm = GPIO.PWM(SERVO_PIN, 50)
        
    def create_widgets(self):
        """Create GUI elements."""
        # Title
        title = tk.Label(self.root, text="🤖 Robot Component Test Suite", 
                        font=('Arial', 16, 'bold'), bg='#f0f0f0')
        title.pack(pady=10)
        
        # Button frame
        btn_frame = tk.Frame(self.root, bg='#f0f0f0')
        btn_frame.pack(pady=10)
        
        # Test buttons
        buttons = [
            ("Test Left Sensor", self.test_left_sensor, '#4CAF50'),
            ("Test Right Sensor", self.test_right_sensor, '#4CAF50'),
            ("Test Motor", self.test_motor, '#FF9800'),
            ("Test Servo", self.test_servo, '#2196F3'),
            ("Test Both Sensors", self.test_both_sensors, '#9C27B0'),
            ("Integration Test", self.integration_test, '#F44336'),
            ("Run All Tests", self.run_all_tests, '#607D8B')
        ]
        
        for i, (text, command, color) in enumerate(buttons):
            row, col = divmod(i, 2)
            btn = tk.Button(btn_frame, text=text, command=command,
                          bg=color, fg='white', font=('Arial', 10, 'bold'),
                          width=15, height=2)
            btn.grid(row=row, column=col, padx=5, pady=5)
        
        # Status frame
        status_frame = tk.Frame(self.root, bg='#f0f0f0')
        status_frame.pack(fill=tk.X, padx=10)
        
        tk.Label(status_frame, text="Status:", font=('Arial', 12, 'bold'), 
                bg='#f0f0f0').pack(anchor='w')
        
        self.status_label = tk.Label(status_frame, text="Ready", 
                                   font=('Arial', 11), bg='#f0f0f0', fg='green')
        self.status_label.pack(anchor='w')
        
        # Output text area
        tk.Label(self.root, text="Test Results:", font=('Arial', 12, 'bold'), 
                bg='#f0f0f0').pack(anchor='w', padx=10, pady=(10,0))
        
        self.output_text = scrolledtext.ScrolledText(self.root, height=15, 
                                                   font=('Courier', 9))
        self.output_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Control buttons
        control_frame = tk.Frame(self.root, bg='#f0f0f0')
        control_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Button(control_frame, text="Clear Output", command=self.clear_output,
                 bg='#757575', fg='white').pack(side=tk.LEFT, padx=5)
        
        tk.Button(control_frame, text="Stop All", command=self.stop_all,
                 bg='#d32f2f', fg='white').pack(side=tk.LEFT, padx=5)
        
        tk.Button(control_frame, text="Exit", command=self.exit_app,
                 bg='#424242', fg='white').pack(side=tk.RIGHT, padx=5)
    
    def log(self, message):
        """Add message to output text area."""
        self.output_text.insert(tk.END, message + '\n')
        self.output_text.see(tk.END)
        self.root.update()
    
    def set_status(self, status, color='black'):
        """Update status label."""
        self.status_label.config(text=status, fg=color)
        self.root.update()
    
    def get_distance(self, trig, echo, timeout=0.02):
        """Measure distance using ultrasonic sensor."""
        GPIO.output(trig, True)
        time.sleep(0.0001)  # From 0.00001
        GPIO.output(trig, False)

        start_time = time.time()
        while GPIO.input(echo) == 0:
            if time.time() - start_time > timeout:
                return SAFE_DISTANCE
        pulse_start = time.time()

        while GPIO.input(echo) == 1:
            if time.time() - pulse_start > timeout:
                return SAFE_DISTANCE
        pulse_end = time.time()

        distance = (pulse_end - pulse_start) * 17150
        return min(distance, SAFE_DISTANCE)
    
    def calculate_centering_servo(self, left_dist, right_dist):
        """Calculate servo position to center robot between walls."""
        # Calculate the difference between left and right distances
        diff = left_dist - right_dist
        
        # Define centering sensitivity (how much to steer per cm difference)
        sensitivity = 0.1  # Adjust this value to make steering more/less aggressive
        
        # Calculate servo adjustment based on distance difference
        servo_adjustment = diff * sensitivity
        
        # Limit the adjustment to prevent extreme steering
        max_adjustment = 2.0  # Maximum adjustment from center position
        servo_adjustment = max(min(servo_adjustment, max_adjustment), -max_adjustment)
        
        # Calculate final servo position
        target_servo = SERVO_CENTER + servo_adjustment
        
        # Ensure servo position stays within valid range
        target_servo = max(min(target_servo, SERVO_LEFT), SERVO_RIGHT)
        
        return target_servo
    
    def run_test_thread(self, test_func):
        """Run test in separate thread to prevent GUI freezing."""
        if self.is_testing:
            return
        
        def test_wrapper():
            self.is_testing = True
            try:
                test_func()
            finally:
                self.is_testing = False
                self.set_status("Ready", 'green')
        
        threading.Thread(target=test_wrapper, daemon=True).start()
    
    def test_left_sensor(self):
        self.run_test_thread(self._test_left_sensor)
    
    def _test_left_sensor(self):
        self.set_status("Testing Left Sensor...", 'blue')
        self.log("🔍 Testing Left Ultrasonic Sensor")
        
        distances = []
        for i in range(5):
            distance = self.get_distance(LEFT_TRIG, LEFT_ECHO)
            distances.append(distance)
            self.log(f"   Reading {i+1}: {distance:.1f} cm")
            time.sleep(0.2)
        
        avg = sum(distances) / len(distances)
        self.log(f"   Average: {avg:.1f} cm")
        
        if max(distances) - min(distances) < 10:
            self.log("   ✅ Sensor readings consistent")
        else:
            self.log("   ⚠️ Sensor readings inconsistent")
        self.log("")
    
    def test_right_sensor(self):
        self.run_test_thread(self._test_right_sensor)
    
    def _test_right_sensor(self):
        self.set_status("Testing Right Sensor...", 'blue')
        self.log("🔍 Testing Right Ultrasonic Sensor")
        
        distances = []
        for i in range(5):
            distance = self.get_distance(RIGHT_TRIG, RIGHT_ECHO)
            distances.append(distance)
            self.log(f"   Reading {i+1}: {distance:.1f} cm")
            time.sleep(0.2)
        
        avg = sum(distances) / len(distances)
        self.log(f"   Average: {avg:.1f} cm")
        
        if max(distances) - min(distances) < 10:
            self.log("   ✅ Sensor readings consistent")
        else:
            self.log("   ⚠️ Sensor readings inconsistent")
        self.log("")
    
    def test_motor(self):
        self.run_test_thread(self._test_motor)
    
    def _test_motor(self):
        self.set_status("Testing Motor...", 'orange')
        self.log("⚡ Testing Motor Driver")
        
        # Test direction control only first
        self.log("   Testing direction control...")
        GPIO.output(IN1, True)
        GPIO.output(IN2, False)
        self.log(f"   Forward: IN1={GPIO.input(IN1)}, IN2={GPIO.input(IN2)}")
        time.sleep(1)
        
        GPIO.output(IN1, False)
        GPIO.output(IN2, False)
        self.log(f"   Stop: IN1={GPIO.input(IN1)}, IN2={GPIO.input(IN2)}")
        time.sleep(0.5)
        
        # Test PWM with motor actually running
        self.log("   Testing PWM speeds with motor running...")
        self.motor_pwm.start(0)
        
        # Set direction to forward for actual motor movement
        GPIO.output(IN1, True)
        GPIO.output(IN2, False)
        self.log("   Motor direction set to FORWARD")
        
        speeds = [10, 20, 30, 40, 50]
        for speed in speeds:
            self.log(f"   Running at {speed}% speed")
            self.motor_pwm.ChangeDutyCycle(speed)
            time.sleep(1)  # Longer delay to actually see/hear motor
        
        # Stop motor properly
        self.motor_pwm.ChangeDutyCycle(0)
        GPIO.output(IN1, False)
        GPIO.output(IN2, False)
        self.log("   Motor stopped")
        
        self.log("   ✅ Motor test complete")
        self.log("")
    
    def test_servo(self):
        self.run_test_thread(self._test_servo)
    
    def _test_servo(self):
        self.set_status("Testing Servo...", 'blue')
        self.log("🎯 Testing Servo Motor")
        
        self.servo_pwm.start(SERVO_CENTER)
        time.sleep(0.5)
        
        positions = [
            ("Center", SERVO_CENTER),
            ("Left", SERVO_LEFT),
            ("Center", SERVO_CENTER),
            ("Right", SERVO_RIGHT),
            ("Center", SERVO_CENTER)
        ]
        
        for pos_name, duty_cycle in positions:
            self.log(f"   Moving to {pos_name} ({duty_cycle})")
            self.servo_pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(1)
        
        self.log("   ✅ Servo test complete")
        self.log("")
    
    def test_both_sensors(self):
        self.run_test_thread(self._test_both_sensors)
    
    def _test_both_sensors(self):
        self.set_status("Testing Both Sensors...", 'purple')
        self.log("👁️ Testing Both Sensors")
        
        for i in range(10):
            left_dist = self.get_distance(LEFT_TRIG, LEFT_ECHO)
            right_dist = self.get_distance(RIGHT_TRIG, RIGHT_ECHO)
            
            if left_dist < 50 and right_dist < 50:
                action = "STOP/REVERSE"
            elif left_dist < 50:
                action = "STEER RIGHT"
            elif right_dist < 50:
                action = "STEER LEFT"
            else:
                action = "GO STRAIGHT"
            
            self.log(f"   L:{left_dist:5.1f}cm R:{right_dist:5.1f}cm -> {action}")
            time.sleep(0.3)
        
        self.log("   ✅ Both sensors test complete")
        self.log("")
    
    def integration_test(self):
        self.run_test_thread(self._integration_test)
    
    def _integration_test(self):
        self.set_status("Integration Test Running...", 'red')
        self.log("🔧 System Integration Test with Wall Centering (10 seconds)")
        
        # Start systems
        self.servo_pwm.start(SERVO_CENTER)
        self.motor_pwm.start(30)        # Start at 30% speed
        GPIO.output(IN1, True)
        GPIO.output(IN2, False)
        self.motor_pwm.ChangeDutyCycle(25)  # Then reduce to 25%
        
        start_time = time.time()
        while time.time() - start_time < 10:
            left_dist = self.get_distance(LEFT_TRIG, LEFT_ECHO)
            right_dist = self.get_distance(RIGHT_TRIG, RIGHT_ECHO)
            
            # Wall centering mode: both sensors detect walls within 60cm
            if left_dist < 60 and right_dist < 60:
                # Calculate servo position for centering
                target_servo = self.calculate_centering_servo(left_dist, right_dist)
                self.servo_pwm.ChangeDutyCycle(target_servo)
                
                GPIO.output(IN1, True)
                GPIO.output(IN2, False)
                self.motor_pwm.ChangeDutyCycle(20)  # Slower speed for precise centering
                
                # Calculate distance difference for logging
                diff = abs(left_dist - right_dist)
                action = f"CENTERING (servo:{target_servo:.2f}, diff:{diff:.1f}cm)"
                
            # Single obstacle avoidance
            elif left_dist < 60 and right_dist > 60:
                GPIO.output(IN1, True)
                GPIO.output(IN2, False)
                self.servo_pwm.ChangeDutyCycle(SERVO_RIGHT)
                self.motor_pwm.ChangeDutyCycle(30)  # Slow down while steering
                action = "RIGHT (AVOID)"
                
            elif right_dist < 60 and left_dist > 60:
                self.motor_pwm.ChangeDutyCycle(50)  # Slow down while steering
                action = "LEFT (AVOID)"
                GPIO.output(IN1, True)
                GPIO.output(IN2, True) 
                time.sleep(0.3)     
                self.servo_pwm.ChangeDutyCycle(SERVO_RIGHT)
                GPIO.output(IN1, False)
                GPIO.output(IN2, True)
                time.sleep(0.8)
                self.servo_pwm.ChangeDutyCycle(SERVO_LEFT)
                GPIO.output(IN1, True)
                GPIO.output(IN2, False)
                time.sleep(1)
                self.servo_pwm.ChangeDutyCycle(SERVO_CENTER)
                time.sleep(1)
                
            # Clear path
            else:
                GPIO.output(IN1, True)
                GPIO.output(IN2, False)
                self.servo_pwm.ChangeDutyCycle(SERVO_CENTER)
                self.motor_pwm.ChangeDutyCycle(30)  # Normal speed
                action = "STRAIGHT"
            
            elapsed = time.time() - start_time
            self.log(f"   {elapsed:4.1f}s: L={left_dist:5.1f} R={right_dist:5.1f} -> {action}")
            time.sleep(0.1)  # Fast response time
        
        # Stop everything
        self.motor_pwm.ChangeDutyCycle(0)
        GPIO.output(IN1, False)
        GPIO.output(IN2, False)
        self.servo_pwm.ChangeDutyCycle(SERVO_CENTER)
        
        self.log("   ✅ Integration test with wall centering complete")
        self.log("")
    
    def run_all_tests(self):
        self.run_test_thread(self._run_all_tests)
    
    def _run_all_tests(self):
        self.set_status("Running All Tests...", 'navy')
        self.log("🎉 Running Complete Test Suite")
        self.log("="*40)
        
        self._test_left_sensor()
        self._test_right_sensor()
        self._test_motor()
        self._test_servo()
        self._test_both_sensors()
        self._integration_test()
        
        self.log("🎉 All Tests Complete!")
        self.log("="*40)
    
    def clear_output(self):
        """Clear the output text area."""
        self.output_text.delete(1.0, tk.END)
    
    def stop_all(self):
        """Emergency stop - turn off all outputs."""
        try:
            self.motor_pwm.ChangeDutyCycle(0)
            GPIO.output(IN1, False)
            GPIO.output(IN2, False)
            self.servo_pwm.ChangeDutyCycle(SERVO_CENTER)
            self.set_status("Emergency Stop Activated", 'red')
            self.log("🛑 Emergency Stop - All outputs disabled")
        except:
            pass
    
    def exit_app(self):
        """Clean exit."""
        try:
            self.motor_pwm.stop()
            self.servo_pwm.stop()
            GPIO.cleanup()
        except:
            pass
        self.root.quit()
    
    def run(self):
        """Start the GUI."""
        self.root.protocol("WM_DELETE_WINDOW", self.exit_app)
        self.root.mainloop()

if __name__ == '__main__':
    app = RobotTestGUI()
    app.run()