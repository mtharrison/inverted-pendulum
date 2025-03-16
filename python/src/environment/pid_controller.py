import numpy as np
import time


class PIDController:
    """
    PID controller implementation for the inverted pendulum balancing task.
    """
    def __init__(self, kp=5.0, ki=0.0, kd=0.5, setpoint=0.0, output_limits=(-1.0, 1.0)):
        """
        Initialize PID controller with given parameters.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            setpoint: Target value (typically 0 for upright position)
            output_limits: Tuple of (min, max) output values
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        
        # Internal state
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # For debugging/tuning
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
    
    def reset(self):
        """Reset the controller state."""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
    
    def compute(self, measurement, current_time=None):
        """
        Compute PID control output value for given reference input and measurement.
        
        Args:
            measurement: The measured value (typically pendulum angle)
            current_time: Current time (used for calculating dt)
            
        Returns:
            Control output value
        """
        if current_time is None:
            current_time = time.time()
        
        # Time delta
        dt = current_time - self.last_time
        if dt <= 0.0:
            dt = 0.01  # Failsafe
        
        # Calculate error
        error = self.setpoint - measurement
        
        # Normalize angle error to [-π, π]
        if abs(error) > np.pi:
            error = error - np.sign(error) * 2 * np.pi
        
        # Calculate terms
        self.p_term = self.kp * error
        
        # Integral with anti-windup
        self.integral += error * dt
        self.i_term = self.ki * self.integral
        
        # Derivative (avoid derivative kick by using measurement derivative)
        if dt > 0:
            d_input = (error - self.last_error) / dt
            self.d_term = self.kd * d_input
        
        # Store values for next iteration
        self.last_error = error
        self.last_time = current_time
        
        # Calculate total output
        output = self.p_term + self.i_term + self.d_term
        
        # Apply output limits
        if self.output_limits:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        return output


class BalanceController:
    """
    Controller that combines a PID controller for balance with a position controller.
    This enables balancing the pendulum at a specific cart position.
    """
    def __init__(self, 
                 angle_kp=5.0, angle_ki=0.0, angle_kd=1.0,
                 position_kp=0.2, position_ki=0.0, position_kd=0.1,
                 output_limits=(-1.0, 1.0)):
        # Angle controller (primary)
        self.angle_pid = PIDController(
            kp=angle_kp, 
            ki=angle_ki, 
            kd=angle_kd,
            setpoint=0.0,  # Upright position
            output_limits=output_limits
        )
        
        # Position controller (secondary) - provides setpoint adjustments for angle
        self.position_pid = PIDController(
            kp=position_kp,
            ki=position_ki,
            kd=position_kd,
            setpoint=0.0,  # Center position
            output_limits=(-0.2, 0.2)  # Small adjustment to angle setpoint
        )
        
        self.output_limits = output_limits
    
    def reset(self):
        """Reset both controllers."""
        self.angle_pid.reset()
        self.position_pid.reset()
    
    def compute(self, angle, angle_velocity, position, position_velocity, current_time=None):
        """
        Compute control action to balance pendulum at desired position.
        
        Args:
            angle: Current pendulum angle (0 is upright)
            angle_velocity: Angular velocity
            position: Cart position
            position_velocity: Cart velocity
            current_time: Current time
            
        Returns:
            Control output value (-1 to 1)
        """
        if current_time is None:
            current_time = time.time()
        
        # First compute position correction
        # This adjusts the angle setpoint slightly to move the cart to center
        position_correction = self.position_pid.compute(position, current_time)
        
        # Apply the position correction to the angle setpoint
        self.angle_pid.setpoint = position_correction
        
        # Now compute the primary control signal based on angle
        output = self.angle_pid.compute(angle, current_time)
        
        # We can add a damping term based on angular velocity if needed
        # output -= angle_velocity * 0.1  # Simple damping
        
        # Apply output limits
        output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        return output