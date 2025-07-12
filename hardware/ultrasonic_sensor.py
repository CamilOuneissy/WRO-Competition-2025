# hardware/ultrasonic_sensor.py

import time
import logging
import numpy as np
import threading
from collections import deque

class UltrasonicSensor:
    """Enhanced ultrasonic sensor with filtering and error handling"""
    
    def __init__(self, trig_pin, echo_pin, gpio_controller, name="sensor", max_distance=400):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.gpio = gpio_controller
        self.name = name
        self.max_distance = max_distance  # Maximum reliable distance in cm
        
        # Filtering parameters
        self.readings_buffer = deque(maxlen=10)
        self.last_valid_reading = None
        self.reading_timeout = 0.05  # 50ms timeout
        
        # Statistics
        self.total_readings = 0
        self.failed_readings = 0
        self.last_reading_time = 0
        
        # Continuous reading thread
        self.continuous_mode = False
        self.reading_thread = None
        self.latest_reading = None
        self.reading_lock = threading.Lock()
        
        self.initialize()
        logging.info(f"Ultrasonic sensor '{name}' initialized on pins T:{trig_pin} E:{echo_pin}")
    
    def initialize(self):
        """Initialize GPIO pins for ultrasonic sensor"""
        try:
            self.gpio.setup_pin(self.trig_pin, self.gpio.OUT, initial=self.gpio.LOW)
            self.gpio.setup_pin(self.echo_pin, self.gpio.IN)
            
            # Ensure trigger is low
            self.gpio.output(self.trig_pin, self.gpio.LOW)
            time.sleep(0.1)  # Let sensor settle
            
        except Exception as e:
            logging.error(f"Failed to initialize ultrasonic sensor {self.name}: {e}")
            raise
    
    def get_distance(self):
        """Get single distance measurement with error handling"""
        try:
            self.total_readings += 1
            start_time = time.time()
            
            # Generate trigger pulse
            self.gpio.output(self.trig_pin, self.gpio.HIGH)
            time.sleep(0.00001)  # 10 microsecond pulse
            self.gpio.output(self.trig_pin, self.gpio.LOW)
            
            # Wait for echo start
            pulse_start = time.time()
            timeout = pulse_start + self.reading_timeout
            
            while self.gpio.input(self.echo_pin) == self.gpio.LOW:
                pulse_start = time.time()
                if pulse_start > timeout:
                    self.failed_readings += 1
                    logging.debug(f"Sensor {self.name}: Echo start timeout")
                    return None
            
            # Wait for echo end
            pulse_end = time.time()
            timeout = pulse_end + self.reading_timeout
            
            while self.gpio.input(self.echo_pin) == self.gpio.HIGH:
                pulse_end = time.time()
                if pulse_end > timeout:
                    self.failed_readings += 1
                    logging.debug(f"Sensor {self.name}: Echo end timeout")
                    return None
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = (pulse_duration * 34300) / 2  # Speed of sound = 343 m/s
            
            # Validate reading
            if self._is_valid_reading(distance):
                self.readings_buffer.append(distance)
                self.last_valid_reading = distance
                self.last_reading_time = time.time()
                return round(distance, 1)
            else:
                self.failed_readings += 1
                logging.debug(f"Sensor {self.name}: Invalid reading {distance:.1f}cm")
                return None
                
        except Exception as e:
            self.failed_readings += 1
            logging.error(f"Sensor {self.name} reading error: {e}")
            return None
    
    def _is_valid_reading(self, distance):
        """Validate if distance reading is reasonable"""
        # Check basic range
        if distance < 2 or distance > self.max_distance:
            return False
        
        # Check for obvious outliers if we have previous readings
        if len(self.readings_buffer) > 0:
            recent_readings = list(self.readings_buffer)[-5:]  # Last 5 readings
            if len(recent_readings) >= 3:
                median_recent = np.median(recent_readings)
                if abs(distance - median_recent) > 50:  # More than 50cm difference
                    return False
        
        return True
    
    def get_filtered_distance(self, samples=3):
        """Get distance with multiple samples and filtering"""
        readings = []
        
        for _ in range(samples):
            distance = self.get_distance()
            if distance is not None:
                readings.append(distance)
            time.sleep(0.01)  # Small delay between readings
        
        if len(readings) == 0:
            return None
        elif len(readings) == 1:
            return readings[0]
        else:
            # Return median of valid readings
            return round(np.median(readings), 1)
    
    def get_average_distance(self, samples=5, timeout=1.0):
        """Get average distance over multiple samples with timeout"""
        readings = []
        start_time = time.time()
        
        while len(readings) < samples and (time.time() - start_time) < timeout:
            distance = self.get_distance()
            if distance is not None:
                readings.append(distance)
            time.sleep(0.02)  # 20ms between readings
        
        if readings:
            return round(np.mean(readings), 1)
        return None
    
    def start_continuous_reading(self, interval=0.1):
        """Start continuous distance reading in background"""
        if self.continuous_mode:
            return
        
        self.continuous_mode = True
        self.reading_thread = threading.Thread(
            target=self._continuous_reading_loop, 
            args=(interval,), 
            daemon=True
        )
        self.reading_thread.start()
        logging.info(f"Sensor {self.name}: Continuous reading started")
    
    def stop_continuous_reading(self):
        """Stop continuous distance reading"""
        self.continuous_mode = False
        if self.reading_thread and self.reading_thread.is_alive():
            self.reading_thread.join(timeout=1.0)
        logging.info(f"Sensor {self.name}: Continuous reading stopped")
    
    def _continuous_reading_loop(self, interval):
        """Continuous reading loop running in background thread"""
        while self.continuous_mode:
            try:
                distance = self.get_distance()
                with self.reading_lock:
                    self.latest_reading = distance
                time.sleep(interval)
            except Exception as e:
                logging.error(f"Sensor {self.name} continuous reading error: {e}")
                time.sleep(interval)
    
    def get_latest_reading(self):
        """Get latest reading from continuous mode"""
        if not self.continuous_mode:
            return self.get_distance()
        
        with self.reading_lock:
            return self.latest_reading
    
    def get_sensor_status(self):
        """Get comprehensive sensor status"""
        success_rate = 0
        if self.total_readings > 0:
            success_rate = ((self.total_readings - self.failed_readings) / self.total_readings) * 100
        
        return {
            'name': self.name,
            'pins': {'trigger': self.trig_pin, 'echo': self.echo_pin},
            'total_readings': self.total_readings,
            'failed_readings': self.failed_readings,
            'success_rate': round(success_rate, 1),
            'last_valid_reading': self.last_valid_reading,
            'last_reading_time': self.last_reading_time,
            'continuous_mode': self.continuous_mode,
            'buffer_size': len(self.readings_buffer)
        }
    
    def test_sensor(self, duration=10.0):
        """Test sensor functionality over specified duration"""
        logging.info(f"Testing sensor {self.name} for {duration} seconds...")
        
        start_time = time.time()
        readings = []
        
        while (time.time() - start_time) < duration:
            distance = self.get_distance()
            if distance is not None:
                readings.append(distance)
            time.sleep(0.1)
        
        if len(readings) == 0:
            logging.error(f"Sensor {self.name}: No valid readings during test")
            return False
        
        # Analyze results
        mean_distance = np.mean(readings)
        std_distance = np.std(readings)
        min_distance = min(readings)
        max_distance = max(readings)
        
        logging.info(f"Sensor {self.name} test results:")
        logging.info(f"  Readings: {len(readings)}")
        logging.info(f"  Mean: {mean_distance:.1f}cm")
        logging.info(f"  Std Dev: {std_distance:.1f}cm")
        logging.info(f"  Range: {min_distance:.1f} - {max_distance:.1f}cm")
        
        # Check for reasonable stability (std dev < 10% of mean)
        if std_distance < (mean_distance * 0.1):
            logging.info(f"Sensor {self.name}: Test PASSED - stable readings")
            return True
        else:
            logging.warning(f"Sensor {self.name}: Test FAILED - unstable readings")
            return False
    
    def cleanup(self):
        """Cleanup sensor resources"""
        self.stop_continuous_reading()
        self.gpio.output(self.trig_pin, self.gpio.LOW)
        logging.info(f"Sensor {self.name} cleaned up")