# config/robot_config.py

import json
import logging
from pathlib import Path

class RobotConfig:
    """Centralized configuration management with validation"""
    
    DEFAULT_CONFIG = {
        "camera": {
            "camera_index": 0,
            "frame_width": 640,
            "frame_height": 480,
            "camera_fps": 30,
            "auto_exposure": True,
            "brightness": 50,
            "contrast": 50,
            "saturation": 60
        },
        "gpio": {
            "motor_enable_pin": 18,
            "motor_direction_pin1": 19,
            "motor_direction_pin2": 20,
            "servo_pin": 21,
            "start_button_pin": 2,
            "led_pin": 3,
            "left_ultrasonic_trig": 4,
            "left_ultrasonic_echo": 17,
            "right_ultrasonic_trig": 27,
            "right_ultrasonic_echo": 22
        },
        "motor": {
            "default_speed": 0.4,
            "max_speed": 0.8,
            "min_speed": 0.2,
            "steering_center": 7.5,
            "steering_range": 2.5,
            "acceleration_rate": 0.1
        },
        "navigation": {
            "distance_threshold": 20,
            "corner_speed": 0.3,
            "obstacle_speed": 0.25,
            "max_laps": 3,
            "turn_timeout": 3.0,
            "stuck_threshold": 5.0
        },
        "color_detection": {
            "red": {
                "hue_ranges": [[0, 10], [170, 180]],  # Handle red wraparound
                "sat_range": [100, 255],
                "val_range": [100, 255],
                "min_area": 500
            },
            "green": {
                "hue_ranges": [[35, 85]],
                "sat_range": [100, 255], 
                "val_range": [100, 255],
                "min_area": 500
            },
            "orange": {
                "hue_ranges": [[10, 25]],
                "sat_range": [100, 255],
                "val_range": [100, 255],
                "min_area": 300
            },
            "blue": {
                "hue_ranges": [[100, 130]],
                "sat_range": [100, 255],
                "val_range": [100, 255],
                "min_area": 300
            }
        },
        "debug": {
            "log_level": "INFO",
            "enable_gui": True,
            "save_images": False,
            "log_sensors": True
        }
    }
    
    def __init__(self, config_path="robot_config.json"):
        self.config_path = Path(config_path)
        self.config = self.load_config()
        self.setup_logging()
    
    def load_config(self):
        """Load configuration with fallback to defaults"""
        try:
            if self.config_path.exists():
                with open(self.config_path, 'r') as f:
                    loaded_config = json.load(f)
                # Merge with defaults
                config = self.DEFAULT_CONFIG.copy()
                self._deep_update(config, loaded_config)
                return config
            else:
                logging.info(f"Config file {self.config_path} not found, using defaults")
                return self.DEFAULT_CONFIG.copy()
        except Exception as e:
            logging.error(f"Error loading config: {e}, using defaults")
            return self.DEFAULT_CONFIG.copy()
    
    def save_config(self):
        """Save current configuration"""
        try:
            with open(self.config_path, 'w') as f:
                json.dump(self.config, f, indent=4)
            logging.info(f"Configuration saved to {self.config_path}")
        except Exception as e:
            logging.error(f"Error saving config: {e}")
    
    def _deep_update(self, base_dict, update_dict):
        """Recursively update nested dictionaries"""
        for key, value in update_dict.items():
            if key in base_dict and isinstance(base_dict[key], dict) and isinstance(value, dict):
                self._deep_update(base_dict[key], value)
            else:
                base_dict[key] = value
    
    def setup_logging(self):
        """Setup logging configuration"""
        log_level = self.get('debug', 'log_level', default='INFO')
        
        # Create logs directory if it doesn't exist
        Path('logs').mkdir(exist_ok=True)
        
        logging.basicConfig(
            level=getattr(logging, log_level),
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('logs/robot.log'),
                logging.StreamHandler()
            ]
        )
    
    def get(self, *args, **kwargs):
        """
        Get configuration value with proper handling of all calling patterns.
        
        Usage patterns:
        - get('section') -> returns entire section dict
        - get('section', 'key') -> returns section.key value  
        - get('section', 'key', default=value) -> returns section.key or default
        - get('section', default={}) -> returns section or default dict
        """
        default = kwargs.get('default', None)
        
        if len(args) == 0:
            return self.config
            
        elif len(args) == 1:
            # config.get('section') or config.get('section', default={})
            section = args[0]
            return self.config.get(section, default)
            
        elif len(args) == 2:
            section, key = args
            
            # Handle the case where second argument might be a default value (dict)
            if isinstance(key, dict):
                # config.get('section', {}) - key is actually a default value
                return self.config.get(section, key)
            else:
                # config.get('section', 'key') - normal nested access
                if section in self.config and isinstance(self.config[section], dict):
                    return self.config[section].get(key, default)
                else:
                    return default
                    
        elif len(args) >= 3:
            # Deep nested access: config.get('section', 'subsection', 'key')
            current = self.config
            for key in args:
                if isinstance(current, dict) and key in current:
                    current = current[key]
                else:
                    return default
            return current
            
        else:
            return default
    
    def set(self, value, *keys):
        """Set nested configuration value"""
        config = self.config
        for key in keys[:-1]:
            if key not in config:
                config[key] = {}
            config = config[key]
        config[keys[-1]] = value
    
    def validate_config(self):
        """Validate configuration values"""
        errors = []
        
        # Validate GPIO pins
        gpio_pins = []
        gpio_config = self.get('gpio', default={})
        for key, pin in gpio_config.items():
            if pin in gpio_pins:
                errors.append(f"Duplicate GPIO pin {pin} for {key}")
            gpio_pins.append(pin)
        
        # Validate motor parameters
        motor_config = self.get('motor', default={})
        max_speed = motor_config.get('max_speed', 0.8)
        min_speed = motor_config.get('min_speed', 0.2)
        if max_speed <= min_speed:
            errors.append("max_speed must be greater than min_speed")
        
        # Validate camera settings
        camera_config = self.get('camera', default={})
        width = camera_config.get('frame_width', 640)
        height = camera_config.get('frame_height', 480)
        if width <= 0 or height <= 0:
            errors.append("Invalid camera resolution")
        
        return errors