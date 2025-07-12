# hardware/gpio_controller.py

import logging
import time

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    logging.warning("RPi.GPIO not available, using mock GPIO")

class GPIOController:
    """GPIO abstraction layer with mock support for development"""
    
    def __init__(self, use_mock=None):
        self.use_mock = use_mock if use_mock is not None else not GPIO_AVAILABLE
        self.pwm_instances = {}
        self.setup_pins = {}
        
        if not self.use_mock:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            logging.info("Real GPIO initialized")
        else:
            logging.info("Mock GPIO initialized")
    
    def setup_pin(self, pin, mode, initial=None):
        """Setup a GPIO pin with optional initial value"""
        if not self.use_mock:
            if initial is not None:
                GPIO.setup(pin, mode, initial=initial)
            else:
                GPIO.setup(pin, mode)
        
        self.setup_pins[pin] = {
            'mode': mode,
            'initial': initial
        }
        logging.debug(f"GPIO pin {pin} setup as {mode}")
    
    def output(self, pin, value):
        """Set GPIO pin output"""
        if not self.use_mock:
            GPIO.output(pin, value)
        logging.debug(f"GPIO pin {pin} set to {value}")
    
    def input(self, pin):
        """Read GPIO pin input"""
        if not self.use_mock:
            return GPIO.input(pin)
        return 0  # Mock always returns 0
    
    def setup_pwm(self, pin, frequency):
        """Setup PWM on a pin"""
        if not self.use_mock:
            pwm = GPIO.PWM(pin, frequency)
            self.pwm_instances[pin] = pwm
            logging.debug(f"PWM setup on pin {pin} at {frequency}Hz")
            return pwm
        else:
            # Mock PWM class
            class MockPWM:
                def __init__(self, pin, freq):
                    self.pin = pin
                    self.frequency = freq
                    self.duty_cycle = 0
                    self.is_started = False
                
                def start(self, duty_cycle):
                    self.duty_cycle = duty_cycle
                    self.is_started = True
                    logging.debug(f"Mock PWM pin {self.pin} started at {duty_cycle}%")
                
                def ChangeDutyCycle(self, duty_cycle):
                    self.duty_cycle = duty_cycle
                    logging.debug(f"Mock PWM pin {self.pin} duty cycle: {duty_cycle}%")
                
                def stop(self):
                    self.is_started = False
                    logging.debug(f"Mock PWM pin {self.pin} stopped")
                
                def ChangeFrequency(self, frequency):
                    self.frequency = frequency
                    logging.debug(f"Mock PWM pin {self.pin} frequency: {frequency}Hz")
            
            mock_pwm = MockPWM(pin, frequency)
            self.pwm_instances[pin] = mock_pwm
            return mock_pwm
    
    def wait_for_edge(self, pin, edge, timeout=None):
        """Wait for edge detection on pin"""
        if not self.use_mock:
            try:
                if timeout:
                    return GPIO.wait_for_edge(pin, edge, timeout=int(timeout * 1000))
                else:
                    return GPIO.wait_for_edge(pin, edge)
            except:
                return None
        else:
            # Mock implementation
            if timeout:
                time.sleep(min(timeout, 0.1))
            return pin  # Mock success
    
    def cleanup_pin(self, pin):
        """Cleanup specific pin"""
        if pin in self.pwm_instances:
            try:
                self.pwm_instances[pin].stop()
                del self.pwm_instances[pin]
            except:
                pass
        
        if not self.use_mock:
            try:
                GPIO.cleanup(pin)
            except:
                pass
        
        if pin in self.setup_pins:
            del self.setup_pins[pin]
        
        logging.debug(f"GPIO pin {pin} cleaned up")
    
    def cleanup(self):
        """Cleanup all GPIO resources"""
        # Stop all PWM instances
        for pin, pwm in self.pwm_instances.items():
            try:
                pwm.stop()
            except:
                pass
        
        self.pwm_instances.clear()
        
        # Cleanup GPIO
        if not self.use_mock:
            GPIO.cleanup()
        
        self.setup_pins.clear()
        logging.info("All GPIO resources cleaned up")
    
    def get_mode_constant(self, mode_name):
        """Get GPIO mode constant"""
        if self.use_mock:
            # Mock constants
            constants = {
                'OUT': 'OUT',
                'IN': 'IN',
                'HIGH': 1,
                'LOW': 0,
                'PUD_UP': 'PUD_UP',
                'PUD_DOWN': 'PUD_DOWN',
                'RISING': 'RISING',
                'FALLING': 'FALLING',
                'BOTH': 'BOTH'
            }
            return constants.get(mode_name, mode_name)
        else:
            return getattr(GPIO, mode_name, None)
    
    @property
    def OUT(self):
        return self.get_mode_constant('OUT')
    
    @property
    def IN(self):
        return self.get_mode_constant('IN')
    
    @property
    def HIGH(self):
        return self.get_mode_constant('HIGH')
    
    @property
    def LOW(self):
        return self.get_mode_constant('LOW')