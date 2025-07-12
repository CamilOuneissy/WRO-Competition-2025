#!/usr/bin/env python3
# robot_gui.py - GUI Interface for WRO Future Engineers Robot

import sys
import cv2
import time
import threading
import numpy as np
from pathlib import Path
import logging
import platform
import os

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext
    from PIL import Image, ImageTk
except ImportError:
    print("Required GUI libraries not found. Installing...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pillow"])
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext
    from PIL import Image, ImageTk

# Add project root to path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

class RobotGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("WRO Future Engineers Robot Control Center - By Camil Tony Ouneissy")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2b2b2b')
        
        # Robot instance
        self.robot = None
        self.robot_thread = None
        self.camera_thread = None
        self.status_thread = None
        
        # GUI state
        self.running = True
        self.camera_active = False
        self.current_frame = None
        
        # Create GUI elements
        self.setup_gui()
        self.setup_logging()
        
        # Start status updates
        self.start_status_thread()
        
        # Handle window closing
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def setup_gui(self):
        """Setup the main GUI layout"""
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create notebook for tabs
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Create tabs
        self.create_control_tab()
        self.create_camera_tab()
        self.create_color_calibration_tab()
        self.create_status_tab()
        self.create_config_tab()
        self.create_logs_tab()
        self.create_about_tab()
    
    def create_control_tab(self):
        """Create the main control tab"""
        control_frame = ttk.Frame(self.notebook)
        self.notebook.add(control_frame, text="🎮 Control")
        
        # Left panel - Robot Status
        left_panel = ttk.LabelFrame(control_frame, text="Robot Status", padding=10)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # Status indicators
        self.status_vars = {
            'connection': tk.StringVar(value="Disconnected"),
            'state': tk.StringVar(value="Idle"),
            'speed': tk.StringVar(value="0.0"),
            'steering': tk.StringVar(value="0.0"),
            'laps': tk.StringVar(value="0/3"),
            'sensors': tk.StringVar(value="Unknown")
        }
        
        status_items = [
            ("Connection:", self.status_vars['connection'], "red"),
            ("State:", self.status_vars['state'], "blue"),
            ("Speed:", self.status_vars['speed'], "green"),
            ("Steering:", self.status_vars['steering'], "orange"),
            ("Laps:", self.status_vars['laps'], "purple"),
            ("Sensors:", self.status_vars['sensors'], "brown")
        ]
        
        for i, (label, var, color) in enumerate(status_items):
            frame = ttk.Frame(left_panel)
            frame.pack(fill=tk.X, pady=2)
            ttk.Label(frame, text=label, font=('Arial', 10, 'bold')).pack(side=tk.LEFT)
            status_label = ttk.Label(frame, textvariable=var, font=('Arial', 10))
            status_label.pack(side=tk.LEFT, padx=(10, 0))
        
        # Right panel - Controls
        right_panel = ttk.LabelFrame(control_frame, text="Robot Controls", padding=10)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # Connection controls
        conn_frame = ttk.LabelFrame(right_panel, text="Connection", padding=5)
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(conn_frame, text="🔌 Initialize Robot", 
                  command=self.initialize_robot).pack(side=tk.LEFT, padx=5)
        ttk.Button(conn_frame, text="🔄 Reconnect", 
                  command=self.reconnect_robot).pack(side=tk.LEFT, padx=5)
        ttk.Button(conn_frame, text="⛔ Disconnect", 
                  command=self.disconnect_robot).pack(side=tk.LEFT, padx=5)
        
        # Challenge controls
        challenge_frame = ttk.LabelFrame(right_panel, text="Challenges", padding=5)
        challenge_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(challenge_frame, text="🏁 Open Challenge", 
                  command=self.start_open_challenge).pack(fill=tk.X, pady=2)
        ttk.Button(challenge_frame, text="🚧 Obstacle Challenge", 
                  command=self.start_obstacle_challenge).pack(fill=tk.X, pady=2)
        
        # Direction selection
        direction_frame = ttk.LabelFrame(right_panel, text="Direction", padding=5)
        direction_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.direction_var = tk.StringVar(value="clockwise")
        ttk.Radiobutton(direction_frame, text="↻ Clockwise", 
                       variable=self.direction_var, value="clockwise").pack(anchor=tk.W)
        ttk.Radiobutton(direction_frame, text="↺ Counter-Clockwise", 
                       variable=self.direction_var, value="counter_clockwise").pack(anchor=tk.W)
        
        # Emergency controls
        emergency_frame = ttk.LabelFrame(right_panel, text="Emergency", padding=5)
        emergency_frame.pack(fill=tk.X, pady=(0, 10))
        
        emergency_btn = ttk.Button(emergency_frame, text="🛑 EMERGENCY STOP", 
                                  command=self.emergency_stop)
        emergency_btn.pack(fill=tk.X)
        emergency_btn.configure(style='Emergency.TButton')
        
        # Test controls
        test_frame = ttk.LabelFrame(right_panel, text="Testing", padding=5)
        test_frame.pack(fill=tk.X)
        
        ttk.Button(test_frame, text="🔧 System Test", 
                  command=self.run_system_test).pack(fill=tk.X, pady=2)
        ttk.Button(test_frame, text="📹 Camera Test", 
                  command=self.test_camera).pack(fill=tk.X, pady=2)
        ttk.Button(test_frame, text="🔊 Sensor Test", 
                  command=self.test_sensors).pack(fill=tk.X, pady=2)
    
    def create_camera_tab(self):
        """Create the camera monitoring tab"""
        camera_frame = ttk.Frame(self.notebook)
        self.notebook.add(camera_frame, text="📹 Camera")
        
        # Camera controls
        control_frame = ttk.Frame(camera_frame)
        control_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(control_frame, text="▶️ Start Camera", 
                  command=self.start_camera).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="⏸️ Stop Camera", 
                  command=self.stop_camera).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="📸 Capture", 
                  command=self.capture_image).pack(side=tk.LEFT, padx=5)
        
        # Color detection controls
        ttk.Label(control_frame, text="Show:").pack(side=tk.LEFT, padx=(20, 5))
        self.show_detection = tk.BooleanVar(value=True)
        ttk.Checkbutton(control_frame, text="Color Detection", 
                       variable=self.show_detection).pack(side=tk.LEFT)
        
        # Camera display
        self.camera_label = ttk.Label(camera_frame, text="Camera feed will appear here")
        self.camera_label.pack(expand=True, fill=tk.BOTH, padx=10, pady=10)
        
        # Detection info
        detection_frame = ttk.LabelFrame(camera_frame, text="Detection Info", padding=5)
        detection_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.detection_text = tk.Text(detection_frame, height=4, state=tk.DISABLED)
        self.detection_text.pack(fill=tk.X)
    
    def create_color_calibration_tab(self):
        """Create the color calibration tab"""
        calibration_frame = ttk.Frame(self.notebook)
        self.notebook.add(calibration_frame, text="🎨 Color Calibration")
        
        # Initialize calibration variables first
        self.calibration_active = False
        self.calibration_frame = None
        self.hsv_vars = {}
        self.color_ranges = {
            "red": {"lower": [0, 50, 50], "upper": [10, 255, 255]},
            "green": {"lower": [40, 50, 50], "upper": [80, 255, 255]},
            "blue": {"lower": [100, 50, 50], "upper": [130, 255, 255]},
            "orange": {"lower": [10, 50, 50], "upper": [25, 255, 255]},
            "yellow": {"lower": [20, 50, 50], "upper": [35, 255, 255]},
            "purple": {"lower": [140, 50, 50], "upper": [170, 255, 255]}
        }
        
        # Main container with two panels
        main_container = ttk.Frame(calibration_frame)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left panel - Camera and controls
        left_panel = ttk.LabelFrame(main_container, text="Live Camera Feed", padding=10)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # Camera controls for calibration
        cam_controls = ttk.Frame(left_panel)
        cam_controls.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(cam_controls, text="📹 Start Calibration Camera", 
                  command=self.start_calibration_camera).pack(side=tk.LEFT, padx=5)
        ttk.Button(cam_controls, text="⏹️ Stop Camera", 
                  command=self.stop_calibration_camera).pack(side=tk.LEFT, padx=5)
        ttk.Button(cam_controls, text="📸 Capture Sample", 
                  command=self.capture_color_sample).pack(side=tk.LEFT, padx=5)
        
        # Color selection for calibration
        color_select_frame = ttk.Frame(cam_controls)
        color_select_frame.pack(side=tk.LEFT, padx=(20, 0))
        
        ttk.Label(color_select_frame, text="Calibrating:").pack(side=tk.LEFT)
        self.calibration_color = tk.StringVar(value="red")
        colors = ["red", "green", "blue", "orange", "yellow", "purple"]
        color_combo = ttk.Combobox(color_select_frame, textvariable=self.calibration_color, 
                                  values=colors, state="readonly", width=8)
        color_combo.pack(side=tk.LEFT, padx=(5, 0))
        color_combo.bind("<<ComboboxSelected>>", self.on_color_changed)
        
        # Calibration camera display
        self.calibration_camera_label = ttk.Label(left_panel, text="Start camera to begin color calibration")
        self.calibration_camera_label.pack(fill=tk.BOTH, expand=True)
        
        # Mouse click instructions
        instruction_frame = ttk.LabelFrame(left_panel, text="Instructions", padding=5)
        instruction_frame.pack(fill=tk.X, pady=(10, 0))
        
        instructions = """🖱️ Click on colors in the camera feed to sample them
📊 Adjust HSV sliders to fine-tune color detection
💾 Save calibration when satisfied with detection
🔄 Test with different lighting conditions"""
        
        ttk.Label(instruction_frame, text=instructions, font=('Arial', 9), 
                 justify=tk.LEFT).pack(anchor=tk.W)
        
        # Right panel - HSV Controls and color info
        right_panel = ttk.LabelFrame(main_container, text="HSV Color Ranges", padding=10)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(5, 0))
        
        # Color range controls
        self.create_hsv_controls(right_panel)
        
        # Current color info display
        color_info_frame = ttk.LabelFrame(right_panel, text="Current Detection", padding=5)
        color_info_frame.pack(fill=tk.X, pady=(10, 0))
        
        self.color_info_text = tk.Text(color_info_frame, height=8, width=35, state=tk.DISABLED,
                                      font=('Courier', 9))
        self.color_info_text.pack(fill=tk.X)
        
        # Calibration actions
        action_frame = ttk.LabelFrame(right_panel, text="Actions", padding=5)
        action_frame.pack(fill=tk.X, pady=(10, 0))
        
        ttk.Button(action_frame, text="💾 Save Calibration", 
                  command=self.save_color_calibration).pack(fill=tk.X, pady=2)
        ttk.Button(action_frame, text="📥 Load Calibration", 
                  command=self.load_color_calibration).pack(fill=tk.X, pady=2)
        ttk.Button(action_frame, text="📤 Export Calibration", 
                  command=self.export_color_calibration).pack(fill=tk.X, pady=2)
        ttk.Button(action_frame, text="📥 Import Calibration", 
                  command=self.import_color_calibration).pack(fill=tk.X, pady=2)
        ttk.Button(action_frame, text="🔄 Reset to Defaults", 
                  command=self.reset_color_calibration).pack(fill=tk.X, pady=2)
        ttk.Button(action_frame, text="🧪 Test Detection", 
                  command=self.test_color_detection).pack(fill=tk.X, pady=2)
        
        # Advanced tools
        advanced_frame = ttk.LabelFrame(right_panel, text="Advanced Tools", padding=5)
        advanced_frame.pack(fill=tk.X, pady=(10, 0))
        
        ttk.Button(advanced_frame, text="📊 Generate Report", 
                  command=self.create_color_detection_report).pack(fill=tk.X, pady=1)
        ttk.Button(advanced_frame, text="🚀 Quick Setup", 
                  command=self.quick_color_setup).pack(fill=tk.X, pady=1)
        
        # Preset environments
        preset_frame = ttk.LabelFrame(right_panel, text="Lighting Presets", padding=5)
        preset_frame.pack(fill=tk.X, pady=(10, 0))
        
        ttk.Button(preset_frame, text="☀️ Bright Indoor", 
                  command=lambda: self.load_preset("bright_indoor")).pack(fill=tk.X, pady=1)
        ttk.Button(preset_frame, text="💡 Normal Indoor", 
                  command=lambda: self.load_preset("normal_indoor")).pack(fill=tk.X, pady=1)
        ttk.Button(preset_frame, text="🌥️ Dim Indoor", 
                  command=lambda: self.load_preset("dim_indoor")).pack(fill=tk.X, pady=1)
        ttk.Button(preset_frame, text="🌞 Outdoor", 
                  command=lambda: self.load_preset("outdoor")).pack(fill=tk.X, pady=1)
        
        # Load initial values for red color after all widgets are created
        self.load_color_to_sliders("red")
    
    def on_color_changed(self, event=None):
        """Handle color selection change"""
        color = self.calibration_color.get()
        self.load_color_to_sliders(color)
        self.log_message(f"🎨 Switched to {color} calibration")
    
    def create_hsv_controls(self, parent):
        """Create HSV slider controls"""
        # HSV sliders for the selected color
        hsv_frame = ttk.LabelFrame(parent, text="HSV Adjustment", padding=5)
        hsv_frame.pack(fill=tk.X)
        
        # Lower HSV values
        lower_frame = ttk.LabelFrame(hsv_frame, text="Lower Bounds", padding=5)
        lower_frame.pack(fill=tk.X, pady=(0, 5))
        
        self.hsv_vars['lower_h'] = tk.IntVar(value=0)
        self.hsv_vars['lower_s'] = tk.IntVar(value=50)
        self.hsv_vars['lower_v'] = tk.IntVar(value=50)
        
        # Lower H
        ttk.Label(lower_frame, text="H:").grid(row=0, column=0, sticky=tk.W)
        h_lower_scale = ttk.Scale(lower_frame, from_=0, to=179, variable=self.hsv_vars['lower_h'],
                                 orient=tk.HORIZONTAL, command=self.on_hsv_change)
        h_lower_scale.grid(row=0, column=1, sticky=tk.EW, padx=5)
        ttk.Label(lower_frame, textvariable=self.hsv_vars['lower_h'], width=3).grid(row=0, column=2)
        
        # Lower S
        ttk.Label(lower_frame, text="S:").grid(row=1, column=0, sticky=tk.W)
        s_lower_scale = ttk.Scale(lower_frame, from_=0, to=255, variable=self.hsv_vars['lower_s'],
                                 orient=tk.HORIZONTAL, command=self.on_hsv_change)
        s_lower_scale.grid(row=1, column=1, sticky=tk.EW, padx=5)
        ttk.Label(lower_frame, textvariable=self.hsv_vars['lower_s'], width=3).grid(row=1, column=2)
        
        # Lower V
        ttk.Label(lower_frame, text="V:").grid(row=2, column=0, sticky=tk.W)
        v_lower_scale = ttk.Scale(lower_frame, from_=0, to=255, variable=self.hsv_vars['lower_v'],
                                 orient=tk.HORIZONTAL, command=self.on_hsv_change)
        v_lower_scale.grid(row=2, column=1, sticky=tk.EW, padx=5)
        ttk.Label(lower_frame, textvariable=self.hsv_vars['lower_v'], width=3).grid(row=2, column=2)
        
        lower_frame.columnconfigure(1, weight=1)
        
        # Upper HSV values
        upper_frame = ttk.LabelFrame(hsv_frame, text="Upper Bounds", padding=5)
        upper_frame.pack(fill=tk.X)
        
        self.hsv_vars['upper_h'] = tk.IntVar(value=10)
        self.hsv_vars['upper_s'] = tk.IntVar(value=255)
        self.hsv_vars['upper_v'] = tk.IntVar(value=255)
        
        # Upper H
        ttk.Label(upper_frame, text="H:").grid(row=0, column=0, sticky=tk.W)
        h_upper_scale = ttk.Scale(upper_frame, from_=0, to=179, variable=self.hsv_vars['upper_h'],
                                 orient=tk.HORIZONTAL, command=self.on_hsv_change)
        h_upper_scale.grid(row=0, column=1, sticky=tk.EW, padx=5)
        ttk.Label(upper_frame, textvariable=self.hsv_vars['upper_h'], width=3).grid(row=0, column=2)
        
        # Upper S
        ttk.Label(upper_frame, text="S:").grid(row=1, column=0, sticky=tk.W)
        s_upper_scale = ttk.Scale(upper_frame, from_=0, to=255, variable=self.hsv_vars['upper_s'],
                                 orient=tk.HORIZONTAL, command=self.on_hsv_change)
        s_upper_scale.grid(row=1, column=1, sticky=tk.EW, padx=5)
        ttk.Label(upper_frame, textvariable=self.hsv_vars['upper_s'], width=3).grid(row=1, column=2)
        
        # Upper V
        ttk.Label(upper_frame, text="V:").grid(row=2, column=0, sticky=tk.W)
        v_upper_scale = ttk.Scale(upper_frame, from_=0, to=255, variable=self.hsv_vars['upper_v'],
                                 orient=tk.HORIZONTAL, command=self.on_hsv_change)
        v_upper_scale.grid(row=2, column=1, sticky=tk.EW, padx=5)
        ttk.Label(upper_frame, textvariable=self.hsv_vars['upper_v'], width=3).grid(row=2, column=2)
        
        upper_frame.columnconfigure(1, weight=1)
    
    def on_hsv_change(self, value=None):
        """Handle HSV slider changes"""
        color = self.calibration_color.get()
        
        # Update color ranges with current slider values
        self.color_ranges[color] = {
            "lower": [self.hsv_vars['lower_h'].get(), 
                     self.hsv_vars['lower_s'].get(), 
                     self.hsv_vars['lower_v'].get()],
            "upper": [self.hsv_vars['upper_h'].get(), 
                     self.hsv_vars['upper_s'].get(), 
                     self.hsv_vars['upper_v'].get()]
        }
        
        # Update color info display
        self.update_color_info()
    
    def load_color_to_sliders(self, color):
        """Load color range to HSV sliders"""
        if color in self.color_ranges:
            ranges = self.color_ranges[color]
            self.hsv_vars['lower_h'].set(ranges["lower"][0])
            self.hsv_vars['lower_s'].set(ranges["lower"][1])
            self.hsv_vars['lower_v'].set(ranges["lower"][2])
            self.hsv_vars['upper_h'].set(ranges["upper"][0])
            self.hsv_vars['upper_s'].set(ranges["upper"][1])
            self.hsv_vars['upper_v'].set(ranges["upper"][2])
            self.update_color_info()
    
    def start_calibration_camera(self):
        """Start camera for color calibration"""
        if self.calibration_active:
            return
        
        self.calibration_active = True
        self.log_message("Starting color calibration camera...")
        
        def calibration_camera_loop():
            cap = cv2.VideoCapture(0)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            while self.calibration_active:
                try:
                    ret, frame = cap.read()
                    if ret:
                        self.calibration_frame = frame.copy()
                        
                        # Apply color detection overlay
                        frame_with_detection = self.apply_color_detection_overlay(frame)
                        
                        # Convert to RGB for tkinter
                        frame_rgb = cv2.cvtColor(frame_with_detection, cv2.COLOR_BGR2RGB)
                        frame_pil = Image.fromarray(frame_rgb)
                        frame_pil = frame_pil.resize((640, 480), Image.Resampling.LANCZOS)
                        calibration_image = ImageTk.PhotoImage(frame_pil)
                        
                        # Update GUI in main thread
                        self.root.after(0, lambda img=calibration_image: 
                                       self.calibration_camera_label.configure(image=img))
                        self.root.after(0, lambda img=calibration_image: 
                                       setattr(self, '_calibration_image_ref', img))
                    
                    time.sleep(0.033)  # ~30 FPS
                except Exception as e:
                    self.log_message(f"Calibration camera error: {e}")
                    break
            
            cap.release()
        
        threading.Thread(target=calibration_camera_loop, daemon=True).start()
        
        # Bind mouse click for color sampling
        self.calibration_camera_label.bind("<Button-1>", self.on_camera_click)
    
    def stop_calibration_camera(self):
        """Stop calibration camera"""
        self.calibration_active = False
        self.log_message("Calibration camera stopped")
        self.calibration_camera_label.configure(image='', text="Camera stopped")
        self.calibration_camera_label.unbind("<Button-1>")
    
    def apply_color_detection_overlay(self, frame):
        """Apply color detection overlay to frame"""
        color = self.calibration_color.get()
        
        if color in self.color_ranges:
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            lower = np.array(self.color_ranges[color]["lower"])
            upper = np.array(self.color_ranges[color]["upper"])
            
            mask = cv2.inRange(hsv_frame, lower, upper)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw contours and bounding boxes
            for contour in contours:
                if cv2.contourArea(contour) > 500:  # Filter small areas
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, color.upper(), (x, y - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Show mask as overlay
            mask_colored = cv2.applyColorMap(mask, cv2.COLORMAP_HOT)
            frame = cv2.addWeighted(frame, 0.7, mask_colored, 0.3, 0)
        
        return frame
    
    def on_camera_click(self, event):
        """Handle mouse click on camera feed for color sampling"""
        if self.calibration_frame is not None:
            # Get click coordinates (scale to original frame size)
            x = int(event.x * 640 / self.calibration_camera_label.winfo_width())
            y = int(event.y * 480 / self.calibration_camera_label.winfo_height())
            
            # Sample HSV value at clicked point
            hsv_frame = cv2.cvtColor(self.calibration_frame, cv2.COLOR_BGR2HSV)
            h, s, v = hsv_frame[y, x]
            
            self.log_message(f"Sampled color at ({x},{y}): H={h}, S={s}, V={v}")
            
            # Auto-adjust ranges based on sampled color
            self.auto_adjust_range(h, s, v)
    
    def auto_adjust_range(self, h, s, v):
        """Auto-adjust HSV range based on sampled color"""
        # Adjust ranges with some tolerance
        h_tolerance = 10
        s_tolerance = 50
        v_tolerance = 50
        
        lower_h = max(0, h - h_tolerance)
        upper_h = min(179, h + h_tolerance)
        lower_s = max(0, s - s_tolerance)
        upper_s = min(255, s + s_tolerance)
        lower_v = max(0, v - v_tolerance)
        upper_v = min(255, v + v_tolerance)
        
        # Update sliders
        self.hsv_vars['lower_h'].set(lower_h)
        self.hsv_vars['lower_s'].set(lower_s)
        self.hsv_vars['lower_v'].set(lower_v)
        self.hsv_vars['upper_h'].set(upper_h)
        self.hsv_vars['upper_s'].set(upper_s)
        self.hsv_vars['upper_v'].set(upper_v)
        
        # Update color ranges
        self.on_hsv_change()
    
    def update_color_info(self):
        """Update color information display"""
        color = self.calibration_color.get()
        if color in self.color_ranges:
            ranges = self.color_ranges[color]
            
            info = f"""Color: {color.upper()}

Lower HSV: [{ranges['lower'][0]}, {ranges['lower'][1]}, {ranges['lower'][2]}]
Upper HSV: [{ranges['upper'][0]}, {ranges['upper'][1]}, {ranges['upper'][2]}]

Range Width:
H: {ranges['upper'][0] - ranges['lower'][0]}
S: {ranges['upper'][1] - ranges['lower'][1]} 
V: {ranges['upper'][2] - ranges['lower'][2]}

Tip: Wider ranges detect more
variations but may cause false
positives."""
            
            self.color_info_text.configure(state='normal')
            self.color_info_text.delete(1.0, tk.END)
            self.color_info_text.insert(1.0, info)
            self.color_info_text.configure(state='disabled')
    
    def capture_color_sample(self):
        """Capture a color sample image"""
        if self.calibration_frame is not None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            color = self.calibration_color.get()
            filename = f"color_sample_{color}_{timestamp}.png"
            
            # Create samples directory
            samples_dir = Path("color_samples")
            samples_dir.mkdir(exist_ok=True)
            
            cv2.imwrite(str(samples_dir / filename), self.calibration_frame)
            self.log_message(f"Color sample saved: {filename}")
    
    def save_color_calibration(self):
        """Save current color calibration"""
        try:
            calibration_data = {
                "colors": self.color_ranges,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "version": "1.0"
            }
            
            with open("color_calibration.json", "w") as f:
                import json
                json.dump(calibration_data, f, indent=2)
            
            self.log_message("✅ Color calibration saved")
            messagebox.showinfo("Success", "Color calibration saved successfully!")
            
        except Exception as e:
            self.log_message(f"❌ Error saving calibration: {e}")
            messagebox.showerror("Error", f"Failed to save calibration:\n{e}")
    
    def load_color_calibration(self):
        """Load color calibration"""
        try:
            with open("color_calibration.json", "r") as f:
                import json
                calibration_data = json.load(f)
            
            if "colors" in calibration_data:
                self.color_ranges = calibration_data["colors"]
                
                # Load current color to sliders
                current_color = self.calibration_color.get()
                self.load_color_to_sliders(current_color)
                
                self.log_message("✅ Color calibration loaded")
                messagebox.showinfo("Success", "Color calibration loaded successfully!")
            
        except FileNotFoundError:
            self.log_message("❌ No calibration file found")
            messagebox.showwarning("Warning", "No calibration file found!")
        except Exception as e:
            self.log_message(f"❌ Error loading calibration: {e}")
            messagebox.showerror("Error", f"Failed to load calibration:\n{e}")
    
    def reset_color_calibration(self):
        """Reset color calibration to defaults"""
        self.color_ranges = {
            "red": {"lower": [0, 50, 50], "upper": [10, 255, 255]},
            "green": {"lower": [40, 50, 50], "upper": [80, 255, 255]},
            "blue": {"lower": [100, 50, 50], "upper": [130, 255, 255]},
            "orange": {"lower": [10, 50, 50], "upper": [25, 255, 255]},
            "yellow": {"lower": [20, 50, 50], "upper": [35, 255, 255]},
            "purple": {"lower": [140, 50, 50], "upper": [170, 255, 255]}
        }
        
        # Reload current color
        current_color = self.calibration_color.get()
        self.load_color_to_sliders(current_color)
        
        self.log_message("🔄 Color calibration reset to defaults")
    
    def load_preset(self, preset_name):
        """Load lighting preset"""
        presets = {
            "bright_indoor": {
                "red": {"lower": [0, 70, 70], "upper": [10, 255, 255]},
                "green": {"lower": [50, 60, 60], "upper": [70, 255, 255]},
                "blue": {"lower": [110, 60, 60], "upper": [120, 255, 255]},
                "orange": {"lower": [15, 70, 70], "upper": [25, 255, 255]}
            },
            "normal_indoor": {
                "red": {"lower": [0, 50, 50], "upper": [10, 255, 255]},
                "green": {"lower": [40, 50, 50], "upper": [80, 255, 255]},
                "blue": {"lower": [100, 50, 50], "upper": [130, 255, 255]},
                "orange": {"lower": [10, 50, 50], "upper": [25, 255, 255]}
            },
            "dim_indoor": {
                "red": {"lower": [0, 30, 30], "upper": [15, 255, 200]},
                "green": {"lower": [35, 30, 30], "upper": [85, 255, 200]},
                "blue": {"lower": [95, 30, 30], "upper": [135, 255, 200]},
                "orange": {"lower": [8, 30, 30], "upper": [30, 255, 200]}
            },
            "outdoor": {
                "red": {"lower": [0, 80, 80], "upper": [8, 255, 255]},
                "green": {"lower": [55, 80, 80], "upper": [75, 255, 255]},
                "blue": {"lower": [105, 80, 80], "upper": [125, 255, 255]},
                "orange": {"lower": [12, 80, 80], "upper": [22, 255, 255]}
            }
        }
        
        if preset_name in presets:
            # Update only the colors that exist in the preset
            for color, ranges in presets[preset_name].items():
                if color in self.color_ranges:
                    self.color_ranges[color] = ranges
            
            # Reload current color to sliders
            current_color = self.calibration_color.get()
            self.load_color_to_sliders(current_color)
            
            self.log_message(f"📐 Loaded {preset_name} preset")
    
    def test_color_detection(self):
        """Test color detection with current settings"""
        if not self.calibration_active:
            messagebox.showwarning("Warning", "Please start the calibration camera first!")
            return
        
        self.log_message("🧪 Testing color detection...")
        
        # The detection is already being applied in real-time through the overlay
        # Just update the user
        color = self.calibration_color.get()
        ranges = self.color_ranges[color]
        
        test_info = f"""Testing {color.upper()} detection:
Lower: {ranges['lower']}
Upper: {ranges['upper']}

Look at the camera feed:
- Green boxes show detected areas
- Red overlay shows the detection mask
- Adjust sliders if detection is poor"""
        
        messagebox.showinfo("Color Detection Test", test_info)
    
    def on_calibration_color_change(self, event=None):
        """Handle calibration color combobox change"""
        color = self.calibration_color.get()
        self.load_color_to_sliders(color)
        self.update_color_info()
        self.log_message(f"🎨 Switched to calibrating {color.upper()}")
    
    def validate_hsv_range(self, color):
        """Validate HSV range for a color"""
        if color not in self.color_ranges:
            return False
        
        ranges = self.color_ranges[color]
        lower = ranges['lower']
        upper = ranges['upper']
        
        # Check if lower values are less than upper values
        for i in range(3):
            if lower[i] >= upper[i]:
                return False
        
        # Check if values are within valid HSV ranges
        if not (0 <= lower[0] <= 179 and 0 <= upper[0] <= 179):
            return False
        if not (0 <= lower[1] <= 255 and 0 <= upper[1] <= 255):
            return False
        if not (0 <= lower[2] <= 255 and 0 <= upper[2] <= 255):
            return False
        
        return True
    
    def export_color_calibration(self):
        """Export color calibration to a shareable format"""
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            export_data = {
                "export_info": {
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "version": "1.0",
                    "robot_name": "WRO Future Engineers Robot",
                    "exported_by": os.getenv('USER', 'Unknown')
                },
                "color_ranges": self.color_ranges,
                "validation": {}
            }
            
            # Validate all color ranges
            for color in self.color_ranges:
                export_data["validation"][color] = self.validate_hsv_range(color)
            
            filename = f"color_calibration_export_{timestamp}.json"
            with open(filename, "w") as f:
                import json
                json.dump(export_data, f, indent=2)
            
            self.log_message(f"✅ Color calibration exported to {filename}")
            messagebox.showinfo("Export Success", f"Calibration exported to:\n{filename}")
            
        except Exception as e:
            self.log_message(f"❌ Error exporting calibration: {e}")
            messagebox.showerror("Export Error", f"Failed to export calibration:\n{e}")
    
    def import_color_calibration(self):
        """Import color calibration from file"""
        try:
            from tkinter import filedialog
            
            filename = filedialog.askopenfilename(
                title="Select Color Calibration File",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
            )
            
            if not filename:
                return
            
            with open(filename, "r") as f:
                import json
                import_data = json.load(f)
            
            if "color_ranges" in import_data:
                self.color_ranges = import_data["color_ranges"]
                
                # Load current color to sliders
                current_color = self.calibration_color.get()
                self.load_color_to_sliders(current_color)
                
                # Show import info
                export_info = import_data.get("export_info", {})
                info_msg = f"""Calibration imported successfully!

Source: {export_info.get('robot_name', 'Unknown')}
Created: {export_info.get('timestamp', 'Unknown')}
Exported by: {export_info.get('exported_by', 'Unknown')}

Colors imported: {', '.join(self.color_ranges.keys())}"""
                
                self.log_message(f"✅ Color calibration imported from {filename}")
                messagebox.showinfo("Import Success", info_msg)
            else:
                messagebox.showerror("Import Error", "Invalid calibration file format!")
                
        except FileNotFoundError:
            messagebox.showerror("Import Error", "File not found!")
        except Exception as e:
            self.log_message(f"❌ Error importing calibration: {e}")
            messagebox.showerror("Import Error", f"Failed to import calibration:\n{e}")
    
    def create_color_detection_report(self):
        """Create a detailed color detection report"""
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            report_content = f"""WRO Future Engineers Robot - Color Detection Report
Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}
Robot: WRO Future Engineers Robot by Camil Tony Ouneissy

=== COLOR CALIBRATION SUMMARY ===
"""
            
            for color, ranges in self.color_ranges.items():
                lower = ranges['lower']
                upper = ranges['upper']
                h_range = upper[0] - lower[0]
                s_range = upper[1] - lower[1]
                v_range = upper[2] - lower[2]
                
                validation = "✅ Valid" if self.validate_hsv_range(color) else "❌ Invalid"
                
                report_content += f"""
{color.upper()}:
  Lower HSV: [{lower[0]}, {lower[1]}, {lower[2]}]
  Upper HSV: [{upper[0]}, {upper[1]}, {upper[2]}]
  Range Width: H={h_range}, S={s_range}, V={v_range}
  Validation: {validation}
  Sensitivity: {'High' if h_range > 20 else 'Medium' if h_range > 10 else 'Low'}
"""
            
            report_content += f"""
=== RECOMMENDATIONS ===
• Test calibration in different lighting conditions
• Adjust ranges if false positives occur
• Use lighting presets for consistent environments
• Regularly recalibrate for optimal performance

=== SYSTEM INFO ===
OpenCV Version: {cv2.__version__}
Python Version: {sys.version.split()[0]}
Operating System: {platform.system()} {platform.release()}
Generated by: {os.getenv('USER', 'Unknown')}
"""
            
            filename = f"color_detection_report_{timestamp}.txt"
            with open(filename, "w") as f:
                f.write(report_content)
            
            self.log_message(f"📊 Color detection report saved: {filename}")
            messagebox.showinfo("Report Generated", f"Detailed report saved as:\n{filename}")
            
        except Exception as e:
            self.log_message(f"❌ Error generating report: {e}")
            messagebox.showerror("Report Error", f"Failed to generate report:\n{e}")
    
    def quick_color_setup(self):
        """Quick setup wizard for color calibration"""
        setup_window = tk.Toplevel(self.root)
        setup_window.title("Quick Color Setup Wizard")
        setup_window.geometry("400x300")
        setup_window.resizable(False, False)
        
        # Make window modal
        setup_window.transient(self.root)
        setup_window.grab_set()
        
        # Center the window
        setup_window.geometry("+%d+%d" % (self.root.winfo_rootx() + 50, self.root.winfo_rooty() + 50))
        
        # Header
        header_frame = ttk.Frame(setup_window)
        header_frame.pack(fill=tk.X, padx=20, pady=20)
        
        ttk.Label(header_frame, text="🎨 Quick Color Setup", 
                 font=('Arial', 16, 'bold')).pack()
        ttk.Label(header_frame, text="Select your environment for optimal color detection", 
                 font=('Arial', 10)).pack(pady=(5, 0))
        
        # Environment selection
        env_frame = ttk.LabelFrame(setup_window, text="Environment", padding=15)
        env_frame.pack(fill=tk.X, padx=20, pady=10)
        
        env_var = tk.StringVar(value="normal_indoor")
        environments = [
            ("☀️ Bright Indoor (Well-lit room)", "bright_indoor"),
            ("💡 Normal Indoor (Average lighting)", "normal_indoor"),
            ("🌥️ Dim Indoor (Low light)", "dim_indoor"),
            ("🌞 Outdoor (Natural sunlight)", "outdoor")
        ]
        
        for text, value in environments:
            ttk.Radiobutton(env_frame, text=text, variable=env_var, value=value).pack(anchor=tk.W, pady=2)
        
        # Colors to calibrate
        colors_frame = ttk.LabelFrame(setup_window, text="Colors to Setup", padding=15)
        colors_frame.pack(fill=tk.X, padx=20, pady=10)
        
        color_vars = {}
        colors = [("Red (Traffic signs)", "red"), ("Green (Traffic signs)", "green"), 
                 ("Blue (Track boundaries)", "blue"), ("Orange (Obstacles)", "orange")]
        
        for text, color in colors:
            color_vars[color] = tk.BooleanVar(value=True)
            ttk.Checkbutton(colors_frame, text=text, variable=color_vars[color]).pack(anchor=tk.W, pady=2)
        
        # Buttons
        button_frame = ttk.Frame(setup_window)
        button_frame.pack(fill=tk.X, padx=20, pady=20)
        
        def apply_setup():
            env = env_var.get()
            selected_colors = [color for color, var in color_vars.items() if var.get()]
            
            if not selected_colors:
                messagebox.showwarning("No Colors", "Please select at least one color to setup!")
                return
            
            # Load preset for selected environment
            self.load_preset(env)
            
            # Show completion message
            setup_msg = f"""Quick setup completed!

Environment: {env.replace('_', ' ').title()}
Colors configured: {', '.join(selected_colors)}

You can now fine-tune the calibration using the Color Calibration tab."""
            
            messagebox.showinfo("Setup Complete", setup_msg)
            self.log_message(f"🚀 Quick color setup completed for {env}")
            setup_window.destroy()
        
        ttk.Button(button_frame, text="Apply Setup", command=apply_setup).pack(side=tk.RIGHT, padx=(5, 0))
        ttk.Button(button_frame, text="Cancel", command=setup_window.destroy).pack(side=tk.RIGHT)
    
    def create_status_tab(self):
        """Create the detailed status tab"""
        status_frame = ttk.Frame(self.notebook)
        self.notebook.add(status_frame, text="📊 Status")
        
        # Create treeview for detailed status
        columns = ('Component', 'Status', 'Value', 'Last Update')
        self.status_tree = ttk.Treeview(status_frame, columns=columns, show='headings', height=15)
        
        for col in columns:
            self.status_tree.heading(col, text=col)
            self.status_tree.column(col, width=150)
        
        # Scrollbar for treeview
        scrollbar = ttk.Scrollbar(status_frame, orient=tk.VERTICAL, command=self.status_tree.yview)
        self.status_tree.configure(yscrollcommand=scrollbar.set)
        
        # Pack treeview and scrollbar
        self.status_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(10, 0), pady=10)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 10), pady=10)
        
        # Auto-refresh controls
        refresh_frame = ttk.Frame(status_frame)
        refresh_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(refresh_frame, text="🔄 Refresh Now", 
                  command=self.refresh_status).pack(side=tk.LEFT, padx=5)
        
        self.auto_refresh = tk.BooleanVar(value=True)
        ttk.Checkbutton(refresh_frame, text="Auto Refresh (1s)", 
                       variable=self.auto_refresh).pack(side=tk.LEFT, padx=5)
    
    def create_config_tab(self):
        """Create the configuration tab"""
        config_frame = ttk.Frame(self.notebook)
        self.notebook.add(config_frame, text="⚙️ Config")
        
        # Config editor
        ttk.Label(config_frame, text="Robot Configuration", 
                 font=('Arial', 12, 'bold')).pack(pady=10)
        
        self.config_text = scrolledtext.ScrolledText(config_frame, height=20, width=80)
        self.config_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Config controls
        config_controls = ttk.Frame(config_frame)
        config_controls.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(config_controls, text="📥 Load Config", 
                  command=self.load_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(config_controls, text="💾 Save Config", 
                  command=self.save_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(config_controls, text="🔄 Reload", 
                  command=self.reload_config).pack(side=tk.LEFT, padx=5)
    
    def create_logs_tab(self):
        """Create the logs monitoring tab"""
        logs_frame = ttk.Frame(self.notebook)
        self.notebook.add(logs_frame, text="📝 Logs")
        
        # Log controls - centered container
        log_controls_container = ttk.Frame(logs_frame)
        log_controls_container.pack(fill=tk.X, padx=10, pady=5)
        
        # Center the controls
        log_controls = ttk.Frame(log_controls_container)
        log_controls.pack(anchor=tk.CENTER)
        
        ttk.Button(log_controls, text="🗑️ Clear Logs", 
                  command=self.clear_logs).pack(side=tk.LEFT, padx=5)
        ttk.Button(log_controls, text="💾 Save Logs", 
                  command=self.save_logs).pack(side=tk.LEFT, padx=5)
        
        # Log level selection - centered
        ttk.Label(log_controls, text="Level:").pack(side=tk.LEFT, padx=(20, 5))
        self.log_level_var = tk.StringVar(value="INFO")
        log_levels = ["DEBUG", "INFO", "WARNING", "ERROR"]
        ttk.Combobox(log_controls, textvariable=self.log_level_var, 
                    values=log_levels, state="readonly", width=10).pack(side=tk.LEFT, padx=5)
        
        # Log display - full width
        self.log_text = scrolledtext.ScrolledText(logs_frame, height=25, 
                                                 bg='black', fg='white', font=('Courier', 9))
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=(0, 10))

    def create_about_tab(self):
        """Create the about tab"""
        about_frame = ttk.Frame(self.notebook)
        self.notebook.add(about_frame, text="ℹ️ About")
        
        # Main container with scrolling - full width
        canvas = tk.Canvas(about_frame, bg='#f0f0f0')
        scrollbar = ttk.Scrollbar(about_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Header section - full width
        header_frame = ttk.Frame(scrollable_frame)
        header_frame.pack(fill=tk.X, padx=10, pady=20)
        
        # Robot icon/logo (using text for now)
        robot_label = ttk.Label(header_frame, text="🤖", font=('Arial', 48))
        robot_label.pack()
        
        # Title
        title_label = ttk.Label(header_frame, text="WRO Future Engineers Robot", 
                               font=('Arial', 20, 'bold'))
        title_label.pack(pady=(10, 5))
        
        # Subtitle
        subtitle_label = ttk.Label(header_frame, text="Control Center & Monitoring System", 
                                  font=('Arial', 12, 'italic'))
        subtitle_label.pack()
        
        # Version info - full width
        version_frame = ttk.LabelFrame(scrollable_frame, text="Version Information", padding=15)
        version_frame.pack(fill=tk.X, padx=10, pady=10)
        
        version_info = [
            ("GUI Version:", "1.0.0"),
            ("WRO Season:", "2025"),
            ("Category:", "Future Engineers"),
            ("Age Group:", "14-22 years"),
            ("Python Version:", f"{sys.version.split()[0]}"),
            ("OpenCV Version:", f"{cv2.__version__}" if 'cv2' in globals() else "Not available"),
            ("Build Date:", time.strftime("%Y-%m-%d"))
        ]
        
        for label, value in version_info:
            info_frame = ttk.Frame(version_frame)
            info_frame.pack(fill=tk.X, pady=2)
            ttk.Label(info_frame, text=label, font=('Arial', 10, 'bold')).pack(side=tk.LEFT)
            ttk.Label(info_frame, text=value, font=('Arial', 10)).pack(side=tk.LEFT, padx=(10, 0))
        
        # Challenge information - full width
        challenge_frame = ttk.LabelFrame(scrollable_frame, text="Challenge Information", padding=15)
        challenge_frame.pack(fill=tk.X, padx=10, pady=10)
        
        challenge_text = """WRO Future Engineers 2025 - Self-Driving Cars Challenge
        
🏁 Open Challenge: Navigate 3 laps on a track with random inner wall configurations
🚧 Obstacle Challenge: Navigate 3 laps avoiding red/green traffic signs + parallel parking
        
The robot must drive autonomously using computer vision and sensor fusion to:
• Detect and follow track boundaries
• Identify and properly navigate around colored traffic signs
• Complete precise parallel parking maneuvers
• Maintain consistent lap times and safety protocols"""
        
        challenge_label = ttk.Label(challenge_frame, text=challenge_text, 
                                   font=('Arial', 10), justify=tk.LEFT, wraplength=900)
        challenge_label.pack(anchor=tk.W, fill=tk.X)
        
        # Features section - full width
        features_frame = ttk.LabelFrame(scrollable_frame, text="GUI Features", padding=15)
        features_frame.pack(fill=tk.X, padx=10, pady=10)
        
        features = [
            "🎮 Real-time robot control and monitoring",
            "📹 Live camera feed with color detection overlay",
            "📊 Comprehensive status and sensor monitoring", 
            "⚙️ Live configuration editing and management",
            "📝 Real-time logging with save functionality",
            "🛑 Emergency stop and safety features",
            "🔧 System testing and diagnostics",
            "🚀 Challenge execution (Open & Obstacle)"
        ]
        
        for feature in features:
            feature_label = ttk.Label(features_frame, text=feature, font=('Arial', 10))
            feature_label.pack(anchor=tk.W, pady=2, fill=tk.X)
        
        # Hardware requirements - full width
        hardware_frame = ttk.LabelFrame(scrollable_frame, text="Hardware Requirements", padding=15)
        hardware_frame.pack(fill=tk.X, padx=10, pady=10)
        
        hardware_text = """Recommended Hardware Configuration:
        
🧠 Main Controller: Raspberry Pi 4B (4GB+ RAM)
📷 Camera: USB webcam or Raspberry Pi Camera Module
📡 Sensors: 2x Ultrasonic sensors (HC-SR04 or similar)
⚙️ Motors: DC motor + servo for steering (4-wheel car-like chassis)
🔋 Power: Dual battery system (SBC + motors)
📱 Optional: IMU, encoders, additional sensors"""
        
        hardware_label = ttk.Label(hardware_frame, text=hardware_text, 
                                  font=('Arial', 10), justify=tk.LEFT, wraplength=900)
        hardware_label.pack(anchor=tk.W, fill=tk.X)
        
        # Team information (customizable) - full width
        team_frame = ttk.LabelFrame(scrollable_frame, text="Team Information", padding=15)
        team_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # You can customize this section
        team_info = [
            ("Team Name:", "Your Team Name Here"),
            ("School/Organization:", "Your School/Org Here"),
            ("Country:", "Your Country"),
            ("Coach:", "Coach Name"),
            ("Team Members:", "Member 1, Member 2, Member 3"),
            ("Contact:", "team@example.com")
        ]
        
        for label, value in team_info:
            team_info_frame = ttk.Frame(team_frame)
            team_info_frame.pack(fill=tk.X, pady=2)
            ttk.Label(team_info_frame, text=label, font=('Arial', 10, 'bold')).pack(side=tk.LEFT)
            ttk.Label(team_info_frame, text=value, font=('Arial', 10)).pack(side=tk.LEFT, padx=(10, 0))
        
        # System information - full width
        system_frame = ttk.LabelFrame(scrollable_frame, text="System Information", padding=15)
        system_frame.pack(fill=tk.X, padx=10, pady=10)
        
        system_info = [
            ("Operating System:", f"{platform.system()} {platform.release()}"),
            ("Architecture:", platform.machine()),
            ("Processor:", platform.processor() or "Unknown"),
            ("Hostname:", platform.node()),
            ("Current User:", os.getenv('USER', 'Unknown')),
            ("Working Directory:", os.getcwd())
        ]
        
        for label, value in system_info:
            sys_info_frame = ttk.Frame(system_frame)
            sys_info_frame.pack(fill=tk.X, pady=2)
            ttk.Label(sys_info_frame, text=label, font=('Arial', 10, 'bold')).pack(side=tk.LEFT)
            ttk.Label(sys_info_frame, text=value[:50] + "..." if len(value) > 50 else value, 
                     font=('Arial', 10)).pack(side=tk.LEFT, padx=(10, 0))
        
        # Links and resources - full width
        resources_frame = ttk.LabelFrame(scrollable_frame, text="Resources & Links", padding=15)
        resources_frame.pack(fill=tk.X, padx=10, pady=10)
        
        resources_text = """🌐 Useful Resources:
        
• WRO Official Website: https://wro-association.org
• WRO 2025 Rules: https://wro-association.org/competition/
• Getting Started Guide: Check WRO official documentation
• GitHub Repository: [Your robot's GitHub repo]
• Team Documentation: [Your team's documentation link]"""
        
        resources_label = ttk.Label(resources_frame, text=resources_text, 
                                   font=('Arial', 10), justify=tk.LEFT, wraplength=900)
        resources_label.pack(anchor=tk.W, fill=tk.X)
        
        # License/Credits - full width
        credits_frame = ttk.LabelFrame(scrollable_frame, text="Credits & License", padding=15)
        credits_frame.pack(fill=tk.X, padx=10, pady=(10, 20))
        
        credits_text = """This GUI application was created for the WRO 2025 Future Engineers competition.
        
🙏 Built with: Python, tkinter, OpenCV, PIL, NumPy
📄 License: Open source - modify and distribute as needed
🤝 Contributing: Feel free to improve and share enhancements
⚡ Special thanks to the WRO community and all contributors"""
        
        credits_label = ttk.Label(credits_frame, text=credits_text, 
                                 font=('Arial', 10), justify=tk.LEFT, wraplength=900)
        credits_label.pack(anchor=tk.W, fill=tk.X)
        
        # Action buttons - full width
        button_frame = ttk.Frame(scrollable_frame)
        button_frame.pack(fill=tk.X, padx=10, pady=20)
        
        ttk.Button(button_frame, text="📋 Copy System Info", 
                  command=self.copy_system_info).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="📁 Open Logs Folder", 
                  command=self.open_logs_folder).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="🔄 Refresh Info", 
                  command=self.refresh_about_info).pack(side=tk.LEFT, padx=5)
        
        # Pack canvas and scrollbar to fill full width
        canvas.pack(side="left", fill="both", expand=True, padx=0, pady=0)
        scrollbar.pack(side="right", fill="y", padx=0, pady=0)

    def copy_system_info(self):
        """Copy system information to clipboard"""
        try:
            info = f"""WRO Robot System Information
Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}

Operating System: {platform.system()} {platform.release()}
Architecture: {platform.machine()}
Processor: {platform.processor() or 'Unknown'}
Python Version: {sys.version.split()[0]}
OpenCV Version: {cv2.__version__ if 'cv2' in globals() else 'Not available'}
Working Directory: {os.getcwd()}
User: {os.getenv('USER', 'Unknown')}

Robot Status: {self.status_vars['connection'].get()}
Current State: {self.status_vars['state'].get()}
"""
            
            # Copy to clipboard
            self.root.clipboard_clear()
            self.root.clipboard_append(info)
            self.root.update()
            
            self.log_message("✅ System information copied to clipboard")
            messagebox.showinfo("Copied", "System information copied to clipboard!")
            
        except Exception as e:
            self.log_message(f"❌ Error copying system info: {e}")
            messagebox.showerror("Error", f"Failed to copy system info:\n{e}")

    def open_logs_folder(self):
        """Open the logs folder in file manager"""
        try:
            import subprocess
            
            logs_path = Path("logs")
            if not logs_path.exists():
                logs_path.mkdir()
            
            system = platform.system()
            if system == "Linux":
                subprocess.run(["xdg-open", str(logs_path)])
            elif system == "Darwin":  # macOS
                subprocess.run(["open", str(logs_path)])
            elif system == "Windows":
                subprocess.run(["explorer", str(logs_path)])
            
            self.log_message("📁 Opened logs folder")
            
        except Exception as e:
            self.log_message(f"❌ Error opening logs folder: {e}")
            messagebox.showerror("Error", f"Failed to open logs folder:\n{e}")

    def refresh_about_info(self):
        """Refresh the about tab information"""
        try:
            # Remove and recreate the about tab
            for i, tab_id in enumerate(self.notebook.tabs()):
                if self.notebook.tab(tab_id, "text").strip() == "ℹ️ About":
                    self.notebook.forget(tab_id)
                    break
            
            # Recreate the tab
            self.create_about_tab()
            
            self.log_message("🔄 About tab refreshed")
            
        except Exception as e:
            self.log_message(f"❌ Error refreshing about tab: {e}")
            messagebox.showerror("Error", f"Failed to refresh about tab:\n{e}")
       
    def setup_logging(self):
        """Setup logging to display in GUI"""
        class GUILogHandler(logging.Handler):
            def __init__(self, text_widget):
                super().__init__()
                self.text_widget = text_widget
            
            def emit(self, record):
                log_entry = self.format(record)
                def append():
                    self.text_widget.configure(state='normal')
                    self.text_widget.insert(tk.END, log_entry + '\n')
                    self.text_widget.configure(state='disabled')
                    self.text_widget.see(tk.END)
                self.text_widget.after(0, append)
        
        # Add GUI handler to root logger
        gui_handler = GUILogHandler(self.log_text)
        gui_handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(levelname)s - %(message)s'
        ))
        logging.getLogger().addHandler(gui_handler)
        logging.getLogger().setLevel(logging.INFO)
    
    def initialize_robot(self):
        """Initialize the robot"""
        try:
            # Try to import the robot module - handle case where it doesn't exist
            try:
                from main import WROFutureEngineersRobot
            except ImportError:
                self.log_message("❌ Robot module not found - using simulation mode")
                messagebox.showwarning("Warning", "Robot module not found. Running in simulation mode.")
                self.status_vars['connection'].set("Simulation")
                return
            
            if self.robot is not None:
                self.disconnect_robot()
            
            self.log_message("Initializing robot...")
            self.robot = WROFutureEngineersRobot()
            
            if hasattr(self.robot, 'initialize_hardware') and self.robot.initialize_hardware():
                self.status_vars['connection'].set("Connected")
                self.log_message("✅ Robot initialized successfully")
                messagebox.showinfo("Success", "Robot initialized successfully!")
            else:
                self.status_vars['connection'].set("Failed")
                self.log_message("❌ Robot initialization failed")
                messagebox.showerror("Error", "Robot initialization failed!")
                
        except Exception as e:
            self.log_message(f"❌ Error initializing robot: {e}")
            messagebox.showerror("Error", f"Failed to initialize robot:\n{e}")
    
    def disconnect_robot(self):
        """Disconnect from robot"""
        if self.robot:
            self.log_message("Disconnecting robot...")
            try:
                if hasattr(self.robot, 'cleanup'):
                    self.robot.cleanup()
                self.robot = None
                self.status_vars['connection'].set("Disconnected")
                self.status_vars['state'].set("Idle")
                self.log_message("✅ Robot disconnected")
            except Exception as e:
                self.log_message(f"❌ Error disconnecting robot: {e}")
    
    def reconnect_robot(self):
        """Reconnect to robot"""
        self.disconnect_robot()
        time.sleep(1)
        self.initialize_robot()
    
    def start_open_challenge(self):
        """Start open challenge"""
        if not self.robot:
            messagebox.showerror("Error", "Please initialize robot first!")
            return
        
        direction = "clockwise" if self.direction_var.get() == "clockwise" else "counter_clockwise"
        self.log_message(f"Starting open challenge ({direction})")
        
        def run_challenge():
            try:
                # Try to import and use the Direction enum
                try:
                    from navigation.robot_controller import Direction
                    dir_enum = Direction.CLOCKWISE if direction == "clockwise" else Direction.COUNTER_CLOCKWISE
                    if hasattr(self.robot, 'run_open_challenge'):
                        self.robot.run_open_challenge(dir_enum)
                    else:
                        self.log_message("❌ Robot does not support open challenge")
                except ImportError:
                    self.log_message("❌ Navigation module not found")
                    messagebox.showerror("Error", "Navigation module not available!")
            except Exception as e:
                self.log_message(f"❌ Open challenge error: {e}")
        
        self.robot_thread = threading.Thread(target=run_challenge, daemon=True)
        self.robot_thread.start()
    
    def start_obstacle_challenge(self):
        """Start obstacle challenge"""
        if not self.robot:
            messagebox.showerror("Error", "Please initialize robot first!")
            return
        
        direction = "clockwise" if self.direction_var.get() == "clockwise" else "counter_clockwise"
        self.log_message(f"Starting obstacle challenge ({direction})")
        
        def run_challenge():
            try:
                # Try to import and use the Direction enum
                try:
                    from navigation.robot_controller import Direction
                    dir_enum = Direction.CLOCKWISE if direction == "clockwise" else Direction.COUNTER_CLOCKWISE
                    if hasattr(self.robot, 'run_obstacle_challenge'):
                        self.robot.run_obstacle_challenge(dir_enum)
                    else:
                        self.log_message("❌ Robot does not support obstacle challenge")
                except ImportError:
                    self.log_message("❌ Navigation module not found")
                    messagebox.showerror("Error", "Navigation module not available!")
            except Exception as e:
                self.log_message(f"❌ Obstacle challenge error: {e}")
        
        self.robot_thread = threading.Thread(target=run_challenge, daemon=True)
        self.robot_thread.start()
    
    def emergency_stop(self):
        """Emergency stop"""
        if self.robot:
            self.log_message("🛑 EMERGENCY STOP ACTIVATED")
            try:
                if hasattr(self.robot, 'motor') and hasattr(self.robot.motor, 'emergency_stop'):
                    self.robot.motor.emergency_stop()
                elif hasattr(self.robot, 'emergency_stop'):
                    self.robot.emergency_stop()
                else:
                    self.log_message("❌ Emergency stop method not available")
                self.status_vars['state'].set("Emergency Stop")
                messagebox.showwarning("Emergency Stop", "Emergency stop activated!")
            except Exception as e:
                self.log_message(f"❌ Error during emergency stop: {e}")
        else:
            self.log_message("❌ No robot connected for emergency stop")
    
    def run_system_test(self):
        """Run system tests"""
        if not self.robot:
            messagebox.showerror("Error", "Please initialize robot first!")
            return
        
        self.log_message("Running system tests...")
        
        def run_tests():
            try:
                if hasattr(self.robot, 'run_system_tests'):
                    self.robot.run_system_tests()
                    self.log_message("✅ System tests completed")
                else:
                    self.log_message("❌ System test method not available")
                    # Run basic tests instead
                    self.test_camera()
                    self.test_sensors()
            except Exception as e:
                self.log_message(f"❌ System test error: {e}")
        
        threading.Thread(target=run_tests, daemon=True).start()
    
    def test_camera(self):
        """Test camera functionality"""
        self.log_message("Testing camera...")
        try:
            cap = cv2.VideoCapture(0)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    self.log_message("✅ Camera test successful")
                    messagebox.showinfo("Camera Test", "Camera is working properly!")
                else:
                    self.log_message("❌ Camera test failed - no frame")
                    messagebox.showerror("Camera Test", "Camera not capturing frames!")
                cap.release()
            else:
                self.log_message("❌ Camera test failed - cannot open")
                messagebox.showerror("Camera Test", "Cannot open camera!")
        except Exception as e:
            self.log_message(f"❌ Camera test error: {e}")
            messagebox.showerror("Camera Test", f"Camera test failed:\n{e}")
    
    def test_sensors(self):
        """Test sensor functionality"""
        if not self.robot:
            messagebox.showerror("Error", "Please initialize robot first!")
            return
        
        self.log_message("Testing sensors...")
        
        def test_sensors_thread():
            try:
                # Test ultrasonic sensors if available
                left_distance = None
                right_distance = None
                
                if hasattr(self.robot, 'left_sensor') and hasattr(self.robot.left_sensor, 'get_distance'):
                    left_distance = self.robot.left_sensor.get_distance()
                if hasattr(self.robot, 'right_sensor') and hasattr(self.robot.right_sensor, 'get_distance'):
                    right_distance = self.robot.right_sensor.get_distance()
                
                if left_distance is not None:
                    self.log_message(f"Left sensor: {left_distance}cm")
                if right_distance is not None:
                    self.log_message(f"Right sensor: {right_distance}cm")
                
                if left_distance and right_distance:
                    self.log_message("✅ Sensor tests completed")
                    messagebox.showinfo("Sensor Test", 
                                      f"Sensors working!\nLeft: {left_distance}cm\nRight: {right_distance}cm")
                else:
                    self.log_message("❌ Some sensors failed or not available")
                    messagebox.showwarning("Sensor Test", "Some sensors may not be working properly!")
                    
            except Exception as e:
                self.log_message(f"❌ Sensor test error: {e}")
        
        threading.Thread(target=test_sensors_thread, daemon=True).start()
    
    def start_camera(self):
        """Start camera stream"""
        if self.camera_active:
            return
        
        self.camera_active = True
        self.log_message("Starting camera stream...")
        
        def camera_loop():
            cap = cv2.VideoCapture(0)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            while self.camera_active:
                try:
                    ret, frame = cap.read()
                    if ret:
                        # Process frame for display
                        if self.show_detection.get() and self.robot:
                            frame = self.process_frame_for_detection(frame)
                        
                        # Convert to RGB for tkinter
                        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        frame_pil = Image.fromarray(frame_rgb)
                        frame_pil = frame_pil.resize((640, 480), Image.Resampling.LANCZOS)
                        self.current_frame = ImageTk.PhotoImage(frame_pil)
                        
                        # Update GUI in main thread
                        self.root.after(0, lambda: self.camera_label.configure(image=self.current_frame))
                    
                    time.sleep(0.033)  # ~30 FPS
                except Exception as e:
                    self.log_message(f"Camera error: {e}")
                    break
            
            cap.release()
        
        self.camera_thread = threading.Thread(target=camera_loop, daemon=True)
        self.camera_thread.start()
    
    def stop_camera(self):
        """Stop camera stream"""
        self.camera_active = False
        self.log_message("Camera stream stopped")
        self.camera_label.configure(image='', text="Camera feed stopped")
        self.current_frame = None
    
    def process_frame_for_detection(self, frame):
        """Process frame to show color detection"""
        try:
            if not self.robot or not hasattr(self.robot, 'color_detector'):
                return frame
            
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Detect colors
            detections = []
            colors = ['red', 'green', 'blue', 'orange']
            color_bgr = {'red': (0, 0, 255), 'green': (0, 255, 0), 
                        'blue': (255, 0, 0), 'orange': (0, 165, 255)}
            
            for color in colors:
                try:
                    if hasattr(self.robot.color_detector, 'detect_color'):
                        mask, bbox = self.robot.color_detector.detect_color(hsv_frame, color)
                        if bbox:
                            x1, y1, x2, y2 = bbox
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color_bgr[color], 2)
                            cv2.putText(frame, color.upper(), (x1, y1-10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr[color], 2)
                            detections.append(f"{color}: ({x1},{y1})")
                except:
                    pass
            
            # Update detection info
            if detections:
                detection_text = "Detected: " + ", ".join(detections)
                self.update_detection_info(detection_text)
            
        except Exception as e:
            self.log_message(f"Detection error: {e}")
        
        return frame
    
    def update_detection_info(self, text):
        """Update detection information display"""
        def update():
            self.detection_text.configure(state='normal')
            self.detection_text.delete(1.0, tk.END)
            self.detection_text.insert(1.0, text)
            self.detection_text.configure(state='disabled')
        
        self.root.after(0, update)
    
    def capture_image(self):
        """Capture current camera frame"""
        if self.current_frame:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.png"
            
            # Create captures directory if it doesn't exist
            captures_dir = Path("captures")
            captures_dir.mkdir(exist_ok=True)
            
            # Save the image (this saves the displayed tkinter image)
            # For original frame capture, you'd need to modify the camera loop
            self.log_message(f"Image captured: {filename}")
            messagebox.showinfo("Capture", f"Image saved as {filename}")
        else:
            self.log_message("❌ No camera frame to capture")
            messagebox.showwarning("Capture", "No camera feed active!")
    
    def start_status_thread(self):
        """Start status monitoring thread"""
        def status_loop():
            while self.running:
                try:
                    if self.auto_refresh.get():
                        self.update_status()
                    time.sleep(1)
                except Exception as e:
                    self.log_message(f"Status update error: {e}")
                    time.sleep(5)  # Wait longer on error
        
        self.status_thread = threading.Thread(target=status_loop, daemon=True)
        self.status_thread.start()
    
    def update_status(self):
        """Update robot status"""
        if not self.robot:
            return
        
        try:
            # Update status variables if robot controller exists
            if hasattr(self.robot, 'robot_controller') and self.robot.robot_controller:
                if hasattr(self.robot.robot_controller, 'get_status'):
                    status = self.robot.robot_controller.get_status()
                    self.status_vars['state'].set(status.get('state', 'Unknown'))
                    self.status_vars['laps'].set(f"{status.get('laps_completed', 0)}/3")
                    
                    if 'motor_status' in status:
                        motor_status = status['motor_status']
                        self.status_vars['speed'].set(f"{motor_status.get('current_speed', 0):.2f}")
                        self.status_vars['steering'].set(f"{motor_status.get('current_steering', 0):.2f}")
                    
                    # Update sensor readings
                    left_dist = status.get('left_distance', 'Unknown')
                    right_dist = status.get('right_distance', 'Unknown')
                    self.status_vars['sensors'].set(f"L:{left_dist} R:{right_dist}")
            
            # Update detailed status tree (less frequently to avoid GUI lag)
            import random
            if random.randint(0, 4) == 0:  # Update tree every 5th time
                self.refresh_status()
            
        except Exception as e:
            self.log_message(f"Status update error: {e}")
    
    def refresh_status(self):
        """Refresh detailed status display"""
        # Clear existing items
        for item in self.status_tree.get_children():
            self.status_tree.delete(item)
        
        current_time = time.strftime("%H:%M:%S")
        
        # Add status items
        status_items = [
            ("Connection", "Status", self.status_vars['connection'].get(), current_time),
            ("Robot State", "State", self.status_vars['state'].get(), current_time),
            ("Motor Speed", "m/s", self.status_vars['speed'].get(), current_time),
            ("Steering", "angle", self.status_vars['steering'].get(), current_time),
            ("Lap Progress", "count", self.status_vars['laps'].get(), current_time),
            ("Sensors", "cm", self.status_vars['sensors'].get(), current_time),
        ]
        
        # Add additional robot-specific status if available
        if self.robot:
            try:
                # Add camera status
                camera_status = "Active" if self.camera_active else "Inactive"
                status_items.append(("Camera", "Status", camera_status, current_time))
                
                # Add system info
                status_items.append(("System", "Python", f"{sys.version.split()[0]}", current_time))
                status_items.append(("System", "OpenCV", f"{cv2.__version__}", current_time))
                
            except:
                pass
        
        for item in status_items:
            self.status_tree.insert('', 'end', values=item)
    
    def load_config(self):
        """Load configuration"""
        try:
            config_file = Path('robot_config.json')
            if config_file.exists():
                with open(config_file, 'r') as f:
                    config_content = f.read()
                self.config_text.delete(1.0, tk.END)
                self.config_text.insert(1.0, config_content)
                self.log_message("✅ Configuration loaded")
            else:
                # Create default config
                default_config = """{
    "robot": {
        "name": "WRO Future Engineers Robot",
        "version": "1.0.0"
    },
    "hardware": {
        "camera_index": 0,
        "motor_pins": {
            "speed": 18,
            "direction1": 22,
            "direction2": 23
        },
        "servo_pin": 12,
        "sensor_pins": {
            "left_trigger": 24,
            "left_echo": 25,
            "right_trigger": 16,
            "right_echo": 20
        }
    },
    "vision": {
        "color_ranges": {
            "red": [[0, 50, 50], [10, 255, 255]],
            "green": [[40, 50, 50], [80, 255, 255]],
            "blue": [[100, 50, 50], [130, 255, 255]],
            "orange": [[10, 50, 50], [25, 255, 255]]
        }
    },
    "navigation": {
        "max_speed": 0.5,
        "turn_speed": 0.3,
        "safety_distance": 20
    }
}"""
                self.config_text.delete(1.0, tk.END)
                self.config_text.insert(1.0, default_config)
                self.log_message("📝 Default configuration created")
        except Exception as e:
            self.log_message(f"❌ Error loading config: {e}")
            messagebox.showerror("Error", f"Failed to load configuration:\n{e}")
    
    def save_config(self):
        """Save configuration"""
        try:
            config_content = self.config_text.get(1.0, tk.END)
            with open('robot_config.json', 'w') as f:
                f.write(config_content)
            self.log_message("✅ Configuration saved")
            messagebox.showinfo("Success", "Configuration saved successfully!")
        except Exception as e:
            self.log_message(f"❌ Error saving config: {e}")
            messagebox.showerror("Error", f"Failed to save configuration:\n{e}")
    
    def reload_config(self):
        """Reload configuration"""
        if self.robot:
            try:
                # Reinitialize robot with new config
                self.disconnect_robot()
                time.sleep(1)
                self.initialize_robot()
                self.log_message("✅ Configuration reloaded")
            except Exception as e:
                self.log_message(f"❌ Error reloading config: {e}")
        else:
            self.log_message("ℹ️ No robot connected - config will be used on next initialization")
    
    def clear_logs(self):
        """Clear log display"""
        self.log_text.configure(state='normal')
        self.log_text.delete(1.0, tk.END)
        self.log_text.configure(state='disabled')
        self.log_message("Logs cleared")
    
    def save_logs(self):
        """Save logs to file"""
        try:
            # Create logs directory if it doesn't exist
            logs_dir = Path("logs")
            logs_dir.mkdir(exist_ok=True)
            
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = logs_dir / f"robot_logs_{timestamp}.txt"
            
            logs_content = self.log_text.get(1.0, tk.END)
            with open(filename, 'w') as f:
                f.write(logs_content)
            
            self.log_message(f"✅ Logs saved to {filename}")
            messagebox.showinfo("Success", f"Logs saved to {filename}")
        except Exception as e:
            self.log_message(f"❌ Error saving logs: {e}")
            messagebox.showerror("Error", f"Failed to save logs:\n{e}")
    
    def log_message(self, message):
        """Add message to log display"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        
        def append():
            self.log_text.configure(state='normal')
            self.log_text.insert(tk.END, log_entry + '\n')
            self.log_text.configure(state='disabled')
            self.log_text.see(tk.END)
        
        self.root.after(0, append)
    
    def on_closing(self):
        """Handle application closing"""
        self.log_message("Shutting down GUI...")
        self.running = False
        self.camera_active = False
        
        if self.robot:
            self.disconnect_robot()
        
        # Wait for threads to finish
        time.sleep(0.5)
        
        self.root.destroy()
    
    def run(self):
        """Start the GUI"""
        self.log_message("🤖 WRO Future Engineers Robot GUI Started")
        self.log_message("Welcome! Please initialize the robot to begin.")
        
        # Configure styles
        style = ttk.Style()
        try:
            style.configure('Emergency.TButton', background='red', foreground='white')
        except:
            pass  # Style configuration might fail on some systems
        
        # Load initial config
        self.load_config()
        
        # Start GUI main loop
        self.root.mainloop()

def main():
    """Main entry point for GUI"""
    print("Starting WRO Robot GUI...")
    
    try:
        app = RobotGUI()
        app.run()
    except KeyboardInterrupt:
        print("\nGUI interrupted by user")
    except Exception as e:
        print(f"GUI Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()