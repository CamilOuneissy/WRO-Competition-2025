# vision/color_detector.py

import cv2
import numpy as np
import logging
import time
from pathlib import Path

class EnhancedColorDetector:
    """Enhanced color detector with adaptive thresholding and calibration"""
    
    def __init__(self, config):
        self.config = config
        self.color_configs = config.get('color_detection', {})
        
        # Detection parameters
        self.adaptive_mode = True
        self.use_morphology = True
        self.debug_mode = False
        
        # Calibration data
        self.calibration_samples = {}
        self.lighting_compensation = True
        
        # Performance tracking
        self.detection_stats = {}
        
        logging.info("Enhanced color detector initialized")
    
    def detect_color(self, hsv_image, color_name, roi=None):
        """
        Detect color in HSV image with optional ROI
        
        Args:
            hsv_image: Input image in HSV format
            color_name: Name of color to detect
            roi: Optional region of interest (x, y, w, h)
            
        Returns:
            tuple: (mask, bbox) where bbox is (x1, y1, x2, y2) or None
        """
        if color_name not in self.color_configs:
            logging.warning(f"Color '{color_name}' not configured")
            return np.zeros(hsv_image.shape[:2], dtype=np.uint8), None
        
        # Use ROI if specified
        if roi:
            x, y, w, h = roi
            working_image = hsv_image[y:y+h, x:x+w].copy()
            offset = (x, y)
        else:
            working_image = hsv_image.copy()
            offset = (0, 0)
        
        color_config = self.color_configs[color_name]
        
        # Create mask for all hue ranges
        mask = self._create_color_mask(working_image, color_config)
        
        # Apply morphological operations
        if self.use_morphology:
            mask = self._apply_morphology(mask)
        
        # Find and filter contours
        bbox = self._find_best_contour(mask, color_config, offset)
        
        # Create full-size mask if ROI was used
        if roi:
            full_mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
            full_mask[y:y+h, x:x+w] = mask
            mask = full_mask
        
        # Update detection statistics
        self._update_detection_stats(color_name, bbox is not None)
        
        return mask, bbox
    
    def _create_color_mask(self, hsv_image, color_config):
        """Create binary mask for color detection"""
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        # Process each hue range
        for hue_range in color_config['hue_ranges']:
            hue_low, hue_high = hue_range
            sat_low, sat_high = color_config['sat_range']
            val_low, val_high = color_config['val_range']
            
            # Apply lighting compensation if enabled
            if self.lighting_compensation:
                val_low, val_high = self._adjust_value_range(hsv_image, val_low, val_high)
            
            lower = np.array([hue_low, sat_low, val_low], dtype=np.uint8)
            upper = np.array([hue_high, sat_high, val_high], dtype=np.uint8)
            
            range_mask = cv2.inRange(hsv_image, lower, upper)
            mask = cv2.bitwise_or(mask, range_mask)
        
        return mask
    
    def _adjust_value_range(self, hsv_image, val_low, val_high):
        """Adjust value range based on overall image brightness"""
        mean_brightness = np.mean(hsv_image[:, :, 2])
        
        # Adjust thresholds based on lighting conditions
        if mean_brightness < 100:  # Dark conditions
            val_low = max(0, val_low - 30)
            val_high = max(val_high, 150)
        elif mean_brightness > 200:  # Bright conditions
            val_low = min(255, val_low + 30)
            val_high = 255
        
        return val_low, val_high
    
    def _apply_morphology(self, mask):
        """Apply morphological operations to clean up mask"""
        # Remove noise
        kernel_small = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small, iterations=1)
        
        # Fill holes
        kernel_large = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large, iterations=2)
        
        return mask
    
    def _find_best_contour(self, mask, color_config, offset=(0, 0)):
        """Find best contour and return bounding box"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Filter by area
        min_area = color_config.get('min_area', 300)
        valid_contours = [c for c in contours if cv2.contourArea(c) > min_area]
        
        if not valid_contours:
            return None
        
        # Get largest contour
        largest_contour = max(valid_contours, key=cv2.contourArea)
        
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Apply offset if ROI was used
        x += offset[0]
        y += offset[1]
        
        return (x, y, x + w, y + h)
    
    def _update_detection_stats(self, color_name, detected):
        """Update detection statistics"""
        if color_name not in self.detection_stats:
            self.detection_stats[color_name] = {
                'total_attempts': 0,
                'successful_detections': 0,
                'last_detection_time': None
            }
        
        stats = self.detection_stats[color_name]
        stats['total_attempts'] += 1
        
        if detected:
            stats['successful_detections'] += 1
            stats['last_detection_time'] = time.time()
    
    def detect_multiple_colors(self, hsv_image, color_names=None, roi=None):
        """Detect multiple colors in single image"""
        if color_names is None:
            color_names = list(self.color_configs.keys())
        
        results = {}
        for color_name in color_names:
            mask, bbox = self.detect_color(hsv_image, color_name, roi)
            results[color_name] = {
                'mask': mask,
                'bbox': bbox,
                'detected': bbox is not None
            }
        
        return results
    
    def calibrate_color(self, hsv_image, color_name, bbox_region):
        """Calibrate color detection based on selected region"""
        if bbox_region is None:
            logging.warning(f"No region provided for {color_name} calibration")
            return False
        
        x1, y1, x2, y2 = bbox_region
        sample_region = hsv_image[y1:y2, x1:x2]
        
        if sample_region.size == 0:
            logging.warning(f"Empty region for {color_name} calibration")
            return False
        
        # Calculate HSV statistics
        h_values = sample_region[:, :, 0].flatten()
        s_values = sample_region[:, :, 1].flatten()
        v_values = sample_region[:, :, 2].flatten()
        
        # Remove outliers (keep middle 80%)
        h_values = np.percentile(h_values, [10, 90])
        s_values = np.percentile(s_values, [10, 90])
        v_values = np.percentile(v_values, [10, 90])
        
        # Create new configuration
        new_config = {
            'hue_ranges': [[int(h_values[0]), int(h_values[1])]],
            'sat_range': [int(s_values[0]), int(s_values[1])],
            'val_range': [int(v_values[0]), int(v_values[1])],
            'min_area': self.color_configs.get(color_name, {}).get('min_area', 300)
        }
        
        # Handle red hue wraparound
        if color_name == 'red' and h_values[1] - h_values[0] > 90:
            if h_values[0] < 90:  # Red spans across 0
                new_config['hue_ranges'] = [
                    [0, int(h_values[0] + 10)],
                    [int(h_values[1] - 10), 179]
                ]
        
        self.color_configs[color_name] = new_config
        logging.info(f"Calibrated {color_name}: H={h_values}, S={s_values}, V={v_values}")
        
        return True
    
    def save_calibration(self, filepath="color_calibration.json"):
        """Save current color calibration to file"""
        import json
        
        try:
            with open(filepath, 'w') as f:
                json.dump(self.color_configs, f, indent=2)
            logging.info(f"Color calibration saved to {filepath}")
            return True
        except Exception as e:
            logging.error(f"Failed to save calibration: {e}")
            return False
    
    def load_calibration(self, filepath="color_calibration.json"):
        """Load color calibration from file"""
        import json
        
        try:
            if Path(filepath).exists():
                with open(filepath, 'r') as f:
                    loaded_configs = json.load(f)
                self.color_configs.update(loaded_configs)
                logging.info(f"Color calibration loaded from {filepath}")
                return True
            else:
                logging.warning(f"Calibration file {filepath} not found")
                return False
        except Exception as e:
            logging.error(f"Failed to load calibration: {e}")
            return False
    
    def get_detection_stats(self):
        """Get detection statistics for all colors"""
        stats = {}
        for color_name, color_stats in self.detection_stats.items():
            total = color_stats['total_attempts']
            successful = color_stats['successful_detections']
            
            stats[color_name] = {
                'total_attempts': total,
                'successful_detections': successful,
                'success_rate': round((successful / total * 100) if total > 0 else 0, 1),
                'last_detection_time': color_stats['last_detection_time']
            }
        
        return stats
    
    def reset_stats(self):
        """Reset detection statistics"""
        self.detection_stats.clear()
        logging.info("Detection statistics reset")
    
    def test_color_detection(self, test_image_path, expected_colors):
        """Test color detection on a known image"""
        try:
            # Load test image
            image = cv2.imread(test_image_path)
            if image is None:
                logging.error(f"Could not load test image: {test_image_path}")
                return False
            
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Test each expected color
            results = {}
            for color_name in expected_colors:
                mask, bbox = self.detect_color(hsv_image, color_name)
                results[color_name] = bbox is not None
                
                logging.info(f"Color {color_name}: {'DETECTED' if bbox else 'NOT DETECTED'}")
            
            # Calculate overall success rate
            detected_count = sum(results.values())
            success_rate = detected_count / len(expected_colors) * 100
            
            logging.info(f"Detection test results: {detected_count}/{len(expected_colors)} "
                        f"colors detected ({success_rate:.1f}%)")
            
            return success_rate > 70  # Consider 70% success rate as passing
            
        except Exception as e:
            logging.error(f"Color detection test failed: {e}")
            return False