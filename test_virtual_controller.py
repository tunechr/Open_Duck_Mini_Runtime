#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'mini_bdx_runtime'))

import time

# Test virtual controller directly
print("Testing Virtual Controller...")

try:
    from mini_bdx_runtime.virtual_controller import VirtualController
    
    print("✅ Successfully imported VirtualController")
    
    # Test 1: Head-only mode
    print("\n=== Test 1: Head-only mode (look around every 3 seconds) ===")
    controller = VirtualController(
        command_freq=10,  # Lower frequency for testing
        only_head_control=True,
        look_around_interval=3.0,
        enable_movement=False
    )
    
    print("Virtual controller created. Testing for 15 seconds...")
    start_time = time.time()
    
    while time.time() - start_time < 15:
        commands, buttons, left_trigger, right_trigger = controller.get_last_command()
        
        # Print only when head position changes significantly
        head_pos = commands[4:7]  # head pitch, yaw, roll
        if any(abs(x) > 0.01 for x in head_pos):
            print(f"Head position: Pitch={head_pos[0]:.2f}, Yaw={head_pos[1]:.2f}, Roll={head_pos[2]:.2f}")
        
        time.sleep(0.5)
    
    print("\n=== Test 2: Full movement mode ===")
    controller2 = VirtualController(
        command_freq=10,
        only_head_control=False,
        look_around_interval=2.0,
        movement_interval=5.0,
        enable_movement=True
    )
    
    print("Testing full movement mode for 10 seconds...")
    start_time = time.time()
    
    while time.time() - start_time < 10:
        commands, buttons, left_trigger, right_trigger = controller2.get_last_command()
        
        body_pos = commands[0:3]  # x, y, yaw
        head_pos = commands[4:7]  # head pitch, yaw, roll
        mode = "Head" if controller2.head_control_mode else "Body"
        
        if any(abs(x) > 0.01 for x in body_pos + head_pos):
            print(f"Mode: {mode}, Body: [{body_pos[0]:.2f}, {body_pos[1]:.2f}, {body_pos[2]:.2f}], Head: [{head_pos[0]:.2f}, {head_pos[1]:.2f}, {head_pos[2]:.2f}]")
        
        time.sleep(0.5)
    
    print("\n🎉 Virtual controller test completed successfully!")
    
except ImportError as e:
    print(f"❌ Import error: {e}")
    print("Make sure pygame and numpy are installed:")
    print("pip install pygame numpy")
    
except Exception as e:
    print(f"❌ Error during testing: {e}")
    import traceback
    traceback.print_exc()
