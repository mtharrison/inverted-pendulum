import numpy as np
import time
import math
from typing import Optional, Tuple, Dict, Any
from .physical import InvertedPendulumContinuousControlPhysical
from .pid_controller import PIDController, BalanceController


class PIDControlledPendulum(InvertedPendulumContinuousControlPhysical):
    """
    Extension of the physical pendulum environment that uses PID control for balancing.
    This provides a classical control baseline before trying reinforcement learning.
    """
    
    def __init__(
        self,
        render_mode: str = "human",
        pid_mode: str = "balance",  # 'balance' or 'swing_up'
        pid_params: Optional[Dict[str, float]] = None,
        port: str = "/dev/cu.usbmodem2201"
    ):
        """
        Initialize the PID-controlled pendulum environment.
        
        Args:
            render_mode: Rendering mode (passed to parent)
            pid_mode: Control mode - 'balance' for balancing near equilibrium, 
                     'swing_up' for swing-up and balance
            pid_params: Optional parameters for the PID controller
            port: Serial port for the hardware
        """
        super().__init__(render_mode=render_mode)
        
        # Override serial port if provided
        if port != "/dev/cu.usbmodem2201":
            self.client.close()
            from serial_communication.client import SerialCommunicator
            self.client = SerialCommunicator(port=port)
        
        self.pid_mode = pid_mode
        
        # Default PID parameters
        default_params = {
            'angle_kp': 5.0,
            'angle_ki': 0.0,
            'angle_kd': 1.0,
            'position_kp': 0.2,
            'position_ki': 0.0,
            'position_kd': 0.1
        }
        
        # Override with user parameters if provided
        if pid_params:
            default_params.update(pid_params)
        
        # Create controller
        self.controller = BalanceController(
            angle_kp=default_params['angle_kp'],
            angle_ki=default_params['angle_ki'],
            angle_kd=default_params['angle_kd'],
            position_kp=default_params['position_kp'],
            position_ki=default_params['position_ki'],
            position_kd=default_params['position_kd']
        )
        
        # Performance metrics
        self.time_at_balance = 0  # Time spent balanced
        self.max_time_at_balance = 0  # Maximum continuous time balanced
        self.current_balance_streak = 0  # Current streak of balanced steps
        
        # For swing-up
        self.energy_controller = PIDController(kp=0.5, ki=0.0, kd=0.0, setpoint=1.0)
        
        # Track hardware extents and safe operating range
        self.extent = 1000  # Default value, will be updated during reset
        self.safe_zone_percentage = 0.7  # Use 70% of total range as "safe zone"
        self.danger_zone_percentage = 0.85  # At 85% of range, prioritize centering

    def reset(
        self, seed: Optional[int] = None, options: Optional[dict] = None
    ) -> Tuple[np.ndarray, dict]:
        """Reset the environment and controllers."""
        obs, info = super().reset(seed, options)
        self.controller.reset()
        self.energy_controller.reset()
        
        # Reset metrics
        self.time_at_balance = 0
        self.max_time_at_balance = 0
        self.current_balance_streak = 0
        
        # Get and store the hardware extent from the initial reading
        response = self.client.sense()
        if response and 'extent' in response:
            self.extent = response['extent']
            print(f"Hardware extent detected: {self.extent}")
        
        return obs, info
    
    def step(self, action=None):
        """
        Step the environment using PID control instead of external actions.
        The action parameter is ignored in PID mode.
        """
        # Get current state
        response = self.client.sense()
        if response is None:
            return self.last_step_return
        
        obs = self.convert_observation(response)
        
        # Calculate control signal based on PID mode
        if self.pid_mode == 'balance':
            # Extract pendulum angle from observation
            x = obs[0]  # Cart position normalized
            x_dot = obs[1]  # Cart velocity normalized
            
            # Convert from sin/cos representation back to angle
            theta = math.atan2(obs[3], obs[2])  # atan2(sin, cos)
            theta_dot = obs[4]  # Angular velocity
            
            # Compute control action using PID controller
            control_action = self.controller.compute(
                angle=theta,
                angle_velocity=theta_dot,
                position=x,
                position_velocity=x_dot,
                current_time=time.perf_counter()
            )
            
        elif self.pid_mode == 'swing_up':
            # Simple energy-based swing-up controller
            theta = math.atan2(obs[3], obs[2])
            theta_dot = obs[4]
            x = obs[0]  # Cart position normalized
            
            # Check if we're close to the upright position
            if abs(theta) < 0.3:  # About 17 degrees from vertical
                # Use balance controller when near upright
                control_action = self.controller.compute(
                    angle=theta,
                    angle_velocity=theta_dot,
                    position=obs[0],
                    position_velocity=obs[1],
                    current_time=time.perf_counter()
                )
            else:
                # Better energy-based swing-up controller
                # Phase control approach - push when it makes sense to add energy
                
                # Calculate current energy (simplified)
                energy = 0.5 * (theta_dot ** 2) - math.cos(theta)
                # Energy needed for upright position is about 1
                desired_energy = 1.0
                
                # Energy error
                energy_error = desired_energy - energy
                
                # Center the cart if it's too far from center
                # Get the current position in terms of absolute position
                # x is normalized, so x=1.0 means we're at the extent
                abs_position = abs(x)
                
                # Calculate safe thresholds based on actual hardware extents
                safe_threshold = self.safe_zone_percentage  # e.g., 0.7 means 70% of max travel
                danger_threshold = self.danger_zone_percentage  # e.g., 0.85 means 85% of max travel
                
                # Apply centering force that increases as we get closer to the limits
                center_signal = 0.0
                if abs_position > safe_threshold:
                    # Calculate how far into the danger zone we are (0 to 1)
                    danger_factor = (abs_position - safe_threshold) / (1.0 - safe_threshold)
                    # Strong centering force that scales with how close we are to the limits
                    center_signal = -np.sign(x) * min(0.95, 0.5 + danger_factor * 1.2)
                    
                    # Log extreme positions
                    if abs_position > 0.9:
                        print(f"WARNING: Cart very close to limit: {abs_position:.2f} of max range")
                    
                    # Override with pure centering action if in extreme danger zone
                    if abs_position > danger_threshold:
                        control_action = center_signal
                        # Don't return early, just use the centering action as our control signal
                
                # The basic idea: apply force in the direction that will increase energy
                # When pendulum moving away from down position, push in same direction
                # When pendulum moving toward down position, push in opposite direction
                if abs(theta) < math.pi/2:
                    # Pendulum in upper half - keep it there or help it go up
                    # Apply force away from current angle to create torque
                    control_action = 0.85 * np.sign(theta)
                else:
                    # Pendulum in lower half
                    if energy < desired_energy:
                        # Need more energy - push in direction of angular velocity
                        # This adds energy to the system
                        control_action = 0.85 * np.sign(theta_dot)
                    else:
                        # Too much energy or close to desired - push against velocity
                        # This removes energy from the system
                        control_action = -0.85 * np.sign(theta_dot)
                
                # Add a component of the centering force that increases with proximity to limits
                if abs_position > safe_threshold:
                    # Increase the center_signal weight as we get closer to the limits
                    center_weight = min(0.8, danger_factor * 1.5)
                    control_weight = 1.0 - center_weight
                    control_action = control_action * control_weight + center_signal * center_weight
                    
                    # Debug info for tuning
                    if self.t % 50 == 0 and abs_position > 0.75:
                        print(f"Position: {abs_position:.2f}, Center weight: {center_weight:.2f}, Action: {control_action:.2f}")
        else:
            # Fall back to direct action if mode is invalid
            control_action = 0.0
            
        # Apply the control signal
        self.client.move(float(control_action))
        
        # Wait for next timestep
        while (time.perf_counter() - self.last_time) < 0.01:
            pass
        self.last_time = time.perf_counter()
        
        # Get new state after control action
        response = self.client.sense()
        if response is None:
            return self.last_step_return
        
        # Convert to observation
        new_obs = self.convert_observation(response)
        
        # Check if currently balanced
        theta = math.atan2(new_obs[3], new_obs[2])
        is_balanced = abs(theta) < 0.2 and abs(new_obs[4]) < 3.0  # Within ~11° of vertical, low angular velocity
        
        if is_balanced:
            self.time_at_balance += 1
            self.current_balance_streak += 1
            self.max_time_at_balance = max(self.max_time_at_balance, self.current_balance_streak)
        else:
            self.current_balance_streak = 0
        
        # Check if limits were hit
        limitL = response["limitL"]
        limitR = response["limitR"]
        
        # Check truncation
        truncated = self.t >= self.t_limit
        
        # Termination conditions
        terminated = (limitL or limitR) or abs(new_obs[4]) > 16.0
        truncated = bool(self.t >= self.t_limit)
        self.t += 1
        
        # Reward is 1 for each step balanced, otherwise 0
        reward = 1.0 if is_balanced else 0.0
        
        # Return performance info
        info = {
            'time_balanced': self.time_at_balance,
            'max_continuous_balance': self.max_time_at_balance,
            'is_balanced': is_balanced,
            'control_action': control_action
        }
        
        # Update last step return
        self.last_step_return = (
            new_obs,
            float(reward),
            bool(terminated),
            bool(truncated),
            info,
        )
        
        return new_obs, float(reward), bool(terminated), bool(truncated), info
    
    def tune_controller(self, angle_kp=None, angle_ki=None, angle_kd=None, 
                      position_kp=None, position_ki=None, position_kd=None):
        """Utility method to tune the controller parameters during runtime."""
        if angle_kp is not None:
            self.controller.angle_pid.kp = angle_kp
        if angle_ki is not None:
            self.controller.angle_pid.ki = angle_ki
        if angle_kd is not None:
            self.controller.angle_pid.kd = angle_kd
        if position_kp is not None:
            self.controller.position_pid.kp = position_kp
        if position_ki is not None:
            self.controller.position_pid.ki = position_ki
        if position_kd is not None:
            self.controller.position_pid.kd = position_kd
        
        # Reset integral terms after parameter changes
        self.controller.angle_pid.integral = 0
        self.controller.position_pid.integral = 0
        
        return {
            'angle_kp': self.controller.angle_pid.kp,
            'angle_ki': self.controller.angle_pid.ki,
            'angle_kd': self.controller.angle_pid.kd,
            'position_kp': self.controller.position_pid.kp,
            'position_ki': self.controller.position_pid.ki,
            'position_kd': self.controller.position_pid.kd
        }