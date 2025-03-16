#!/usr/bin/env python3
"""
Test script for the PID-controlled inverted pendulum.
This lets you test if the mechanical system is capable of balancing
before training with reinforcement learning.
"""

import time
import argparse
import numpy as np
import math
import json
from environment.pid_physical import PIDControlledPendulum


def run_balance_test(port, duration=30, mode='balance', pid_params=None):
    """
    Run a balance test with the PID controller.
    
    Args:
        port: Serial port for the hardware
        duration: Test duration in seconds
        mode: 'balance' or 'swing_up'
        pid_params: Dictionary of PID parameters to use
    """
    # Create environment with PID controller
    env = PIDControlledPendulum(
        pid_mode=mode,
        pid_params=pid_params,
        port=port
    )
    
    # Reset the environment
    print("Resetting environment...")
    obs, _ = env.reset()
    print("Environment reset complete.")
    
    print(f"Starting {mode} test for {duration} seconds...")
    print("Initial observation:", format_observation(obs))
    
    # Run for specified duration
    start_time = time.time()
    step_count = 0
    max_continuous_balance = 0
    
    try:
        while time.time() - start_time < duration:
            # Step the environment (PID controller is used internally)
            obs, reward, terminated, truncated, info = env.step()
            step_count += 1
            
            # Print status every second
            if step_count % 100 == 0:
                theta = math.atan2(obs[3], obs[2])
                print(f"Step {step_count}: Angle={theta:.2f}rad ({math.degrees(theta):.1f}°), "
                      f"Angular vel={obs[4]:.2f}, Pos={obs[0]:.2f}, "
                      f"Balanced: {info['is_balanced']}, Control: {info['control_action']:.3f}")
                
            # Update metrics
            max_continuous_balance = max(max_continuous_balance, info['max_continuous_balance'])
            
            if terminated or truncated:
                print("Episode ended early.")
                break
                
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        # Clean up
        env.client.move(0)  # Stop the motor
        
        # Print results
        elapsed = time.time() - start_time
        print(f"\nTest completed after {elapsed:.1f} seconds.")
        print(f"Steps: {step_count}")
        print(f"Time balanced: {info['time_balanced'] / 100:.2f} seconds")
        print(f"Max continuous balance: {max_continuous_balance / 100:.2f} seconds")
        
        # Current PID parameters
        params = env.tune_controller()
        print("\nFinal PID parameters:")
        print(f"  Angle PID: kp={params['angle_kp']}, ki={params['angle_ki']}, kd={params['angle_kd']}")
        print(f"  Position PID: kp={params['position_kp']}, ki={params['position_ki']}, kd={params['position_kd']}")


def format_observation(obs):
    """Format observation values for display."""
    theta = math.atan2(obs[3], obs[2])
    return {
        'position': obs[0],
        'velocity': obs[1],
        'angle': theta,
        'angle_degrees': math.degrees(theta),
        'angular_velocity': obs[4]
    }


def interactive_tuning(port):
    """
    Interactive mode to tune PID parameters in real-time.
    """
    # Load initial parameters from file if it exists
    try:
        with open('pid_params.json', 'r') as f:
            pid_params = json.load(f)
            print("Loaded parameters from pid_params.json")
    except (FileNotFoundError, json.JSONDecodeError):
        pid_params = {
            'angle_kp': 5.0,
            'angle_ki': 0.0,
            'angle_kd': 1.0,
            'position_kp': 0.2,
            'position_ki': 0.0,
            'position_kd': 0.1
        }
    
    # Create environment with PID controller
    env = PIDControlledPendulum(
        pid_mode='balance',
        pid_params=pid_params,
        port=port
    )
    
    # Reset the environment
    print("Resetting environment...")
    env.reset()
    print("Environment reset complete.")
    
    print("\nInteractive PID Tuning Mode")
    print("----------------------------")
    print("Commands:")
    print("  p+/p-: Increase/decrease angle proportional gain")
    print("  i+/i-: Increase/decrease angle integral gain")
    print("  d+/d-: Increase/decrease angle derivative gain")
    print("  x+/x-: Increase/decrease position proportional gain")
    print("  y+/y-: Increase/decrease position integral gain")
    print("  z+/z-: Increase/decrease position derivative gain")
    print("  s: Save parameters to file")
    print("  q: Quit")
    
    running = True
    step_count = 0
    
    try:
        while running:
            # Step the environment
            obs, reward, terminated, truncated, info = env.step()
            step_count += 1
            
            # Print status periodically
            if step_count % 100 == 0:
                theta = math.atan2(obs[3], obs[2])
                print(f"Angle={theta:.2f}rad ({math.degrees(theta):.1f}°), "
                      f"Pos={obs[0]:.2f}, Balanced: {info['is_balanced']}")
            
            # Check for keyboard input (non-blocking)
            import msvcrt if hasattr(msvcrt, 'kbhit') else __import__('select')
            
            # Platform-specific input handling
            has_input = False
            if hasattr(msvcrt, 'kbhit'):  # Windows
                has_input = msvcrt.kbhit()
                if has_input:
                    key = msvcrt.getch().decode('utf-8').lower()
            else:  # Unix-like
                import sys, tty, termios
                old_settings = termios.tcgetattr(sys.stdin)
                try:
                    tty.setcbreak(sys.stdin.fileno())
                    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                        has_input = True
                        key = sys.stdin.read(1).lower()
                finally:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            
            if has_input:
                params = None
                
                # Process the key
                if key == 'q':
                    running = False
                elif key == 'p':
                    params = env.tune_controller(angle_kp=env.controller.angle_pid.kp * 1.2)
                    print(f"Increased angle kp to {params['angle_kp']:.3f}")
                elif key == 'P':
                    params = env.tune_controller(angle_kp=env.controller.angle_pid.kp * 0.8)
                    print(f"Decreased angle kp to {params['angle_kp']:.3f}")
                elif key == 'i':
                    params = env.tune_controller(angle_ki=env.controller.angle_pid.ki + 0.1)
                    print(f"Increased angle ki to {params['angle_ki']:.3f}")
                elif key == 'I':
                    params = env.tune_controller(angle_ki=max(0, env.controller.angle_pid.ki - 0.1))
                    print(f"Decreased angle ki to {params['angle_ki']:.3f}")
                elif key == 'd':
                    params = env.tune_controller(angle_kd=env.controller.angle_pid.kd * 1.2)
                    print(f"Increased angle kd to {params['angle_kd']:.3f}")
                elif key == 'D':
                    params = env.tune_controller(angle_kd=env.controller.angle_pid.kd * 0.8)
                    print(f"Decreased angle kd to {params['angle_kd']:.3f}")
                elif key == 'x':
                    params = env.tune_controller(position_kp=env.controller.position_pid.kp * 1.2)
                    print(f"Increased position kp to {params['position_kp']:.3f}")
                elif key == 'X':
                    params = env.tune_controller(position_kp=env.controller.position_pid.kp * 0.8)
                    print(f"Decreased position kp to {params['position_kp']:.3f}")
                elif key == 's':
                    # Save parameters to file
                    params = env.tune_controller()  # Get current parameters
                    with open('pid_params.json', 'w') as f:
                        json.dump(params, f, indent=2)
                    print(f"Saved parameters to pid_params.json")
                
                # Update our local copy of parameters if changed
                if params:
                    pid_params = params
                    
            if terminated or truncated:
                print("Episode ended, resetting...")
                env.reset()
                
    except KeyboardInterrupt:
        print("\nTuning interrupted by user.")
    finally:
        # Clean up
        env.client.move(0)  # Stop the motor


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Test PID control for inverted pendulum')
    parser.add_argument('--port', type=str, default='/dev/cu.usbmodem2201',
                        help='Serial port for the hardware')
    parser.add_argument('--duration', type=int, default=30,
                        help='Test duration in seconds')
    parser.add_argument('--mode', type=str, choices=['balance', 'swing_up'], default='balance',
                        help='Control mode')
    parser.add_argument('--tune', action='store_true',
                        help='Enter interactive tuning mode')
    
    args = parser.parse_args()
    
    if args.tune:
        interactive_tuning(args.port)
    else:
        run_balance_test(args.port, args.duration, args.mode)