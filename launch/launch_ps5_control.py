#!/usr/bin/env python3

import subprocess
import time
import sys
import os

def main():
    print("🎮 Starting PS5 Arm Controller System...")
    
    # Check if controller is connected
    try:
        result = subprocess.run(['ls', '/dev/input/js*'], 
                              capture_output=True, text=True, shell=True)
        if result.returncode != 0:
            print("❌ No joystick detected! Make sure PS5 controller is connected.")
            print("💡 Try: sudo bluetoothctl and pair your controller")
            return
        
        print(f"✅ Joystick device found: {result.stdout.strip()}")
    except Exception as e:
        print(f"❌ Error checking joystick: {e}")
        return
    
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Go up one level to reach the package root, then into scripts
    controller_script = os.path.join(script_dir, '..', 'scripts', 'ps5_arm_controller.py')
    
    if not os.path.exists(controller_script):
        print(f"❌ Controller script not found at: {controller_script}")
        print("💡 Make sure ps5_arm_controller.py is in the scripts/ directory")
        return
    
    processes = []
    
    try:
        # Start joy node
        print("📡 Starting joy node...")
        joy_process = subprocess.Popen([
            'ros2', 'run', 'joy', 'joy_node', 
            '--ros-args', '-p', 'device:=/dev/input/js0'
        ])
        processes.append(joy_process)
        time.sleep(2)
        
        # Start PS5 arm controller
        print("🤖 Starting PS5 arm controller...")
        controller_process = subprocess.Popen([
            'python3', controller_script
        ])
        processes.append(controller_process)
        
        print("\n✅ PS5 Arm Controller System is running!")
        print("📋 Press Ctrl+C to stop all processes")
        print("\n🎮 PS5 Controller Direct Control Help:")
        print("   📍 ARM JOINT CONTROLS (2 buttons per joint):")
        print("      • Joint 0 (Shoulder Rotation): L1(+) / L2(-)")
        print("      • Joint 1 (Shoulder Pitch)  : R1(+) / R2(-)")
        print("      • Joint 2 (Elbow)           : X(+)  / O(-)")
        print("      • Joint 3 (Wrist Pitch)     : △(+)  / □(-)")
        print("      • Joint 4 (Wrist Roll)      : L3(+) / R3(-)")
        print("")
        print("   🦾 GRIPPER CONTROLS (Dedicated Controller):")
        print("      • Open: ↑ (D-pad Up)")
        print("      • Close: ↓ (D-pad Down)")
        print("")
        print("   ⚙️  SPECIAL:")
        print("      • Emergency Home: SELECT + START")
        print("      • Status Info: PS Button")
        print("      • Fine Control: Hold any shoulder button while moving")
        
        # Wait for processes
        for process in processes:
            process.wait()
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping PS5 Arm Controller System...")
        
        # Terminate all processes
        for process in processes:
            if process.poll() is None:  # Process is still running
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
        
        print("✅ All processes stopped.")
    
    except Exception as e:
        print(f"❌ Error: {e}")
        return

if __name__ == '__main__':
    main() 
