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
                # Energy-based swing-up: E = 0.5*m*l²*θ̇² - m*g*l*cos(θ)
                # We'll use a simplified version with just the cos term
                energy = math.cos(theta)
                # Drive the pendulum by adding energy when needed
                energy_error = 1.0 - energy  # Target energy for upright position
                control_action = np.sign(theta_dot * math.cos(theta)) * min(1.0, abs(energy_error) * 3)
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