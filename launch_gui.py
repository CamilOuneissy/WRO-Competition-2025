#!/usr/bin/env python3
# launch_gui.py - Simple launcher for the robot GUI

import sys
import os
from pathlib import Path

def check_dependencies():
    """Check if required dependencies are installed"""
    required_packages = [
        ('cv2', 'opencv-python'),
        ('PIL', 'Pillow'),
        ('tkinter', 'tkinter (usually included with Python)'),
        ('numpy', 'numpy')
    ]
    
    missing = []
    for import_name, package_name in required_packages:
        try:
            __import__(import_name)
        except ImportError:
            missing.append(package_name)
    
    if missing:
        print("Missing required packages:")
        for package in missing:
            print(f"  - {package}")
        print("\nInstall with: pip install opencv-python pillow numpy")
        return False
    
    return True

def main():
    """Launch the robot GUI"""
    print("WRO Future Engineers Robot GUI Launcher")
    print("=" * 40)
    
    # Check if we're in the right directory
    if not Path("robot_config.json").exists():
        print("❌ Error: robot_config.json not found!")
        print("Please run this script from the WRO robot project directory.")
        return 1
    
    # Check dependencies
    print("Checking dependencies...")
    if not check_dependencies():
        print("❌ Missing dependencies. Please install them first.")
        return 1
    
    print("✅ All dependencies found")
    
    # Launch GUI
    print("Starting Robot GUI...")
    try:
        # Import and run the GUI
        sys.path.insert(0, str(Path.cwd()))
        from robot_gui import RobotGUI
        
        app = RobotGUI()
        app.run()
        
    except KeyboardInterrupt:
        print("\n👋 GUI closed by user")
        return 0
    except ImportError as e:
        print(f"❌ Import error: {e}")
        print("Make sure all robot modules are in the correct directories.")
        return 1
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    print("👋 GUI closed")
    return 0

if __name__ == "__main__":
    sys.exit(main())