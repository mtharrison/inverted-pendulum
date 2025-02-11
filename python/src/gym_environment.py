import gymnasium as gym
import numpy as np
from typing import Optional, Tuple
import time
import math

class InvertedPendulumEnv(gym.Env):
    metadata = {'render_modes': ['human']}

    def __init__(self, client, gui=None, max_episode_steps: int = 500, step_size: float = 0.1):
        super().__init__()
        self.client = client
        self.gui = gui
        self.max_episode_steps = max_episode_steps
        self.step_size = step_size  # Physical distance per action unit
        
        # Discrete action space: -1 (left), 0 (stay), +1 (right)
        self.action_space = gym.spaces.Discrete(3)
        
        # Observation: [theta, theta_dot, cart_position, cart_velocity]
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(5,),
            dtype=np.float32
        )

        # Environment parameters
        self.theta_threshold = 15 * np.pi / 180  # ±15 degrees
        self.cart_position_threshold = 2.4  # meters
        
        self.step_count = 0
        self.episode_data = {
            'timesteps': [],
            'states': [],
            'rewards': [],
            'actions': []
        }
        
    def convert_observation(self, obs: dict) -> np.ndarray:
        theta = obs['theta']
        theta_dot = obs['angular_velocity']
        x = obs['current_position']
        x_dot = obs['velocity']
        return np.array([math.sin(theta), math.cos(theta), theta_dot, x, x_dot], dtype=np.float32)
    
    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None) -> Tuple[np.ndarray, dict]:
        # self.client.reset()
        self.step_count = 0
        self._clear_episode_data()
        
        response = self.client.sense()
        if not response['enabled']:
            self.client.reset()
        
        # wait until resetting=false
        while True:
            response = self.client.sense()
            if not response['resetting']:
                break
            time.sleep(1)
            
        time.sleep(4)
        
        obs = self.convert_observation(self.client.sense())
        
        if self.gui:
            self.gui.on_episode_reset()
        
        return obs, {}

    def step(self, action: np.int8) -> Tuple[np.ndarray, float, bool, bool, dict]:
        self.client.move(300 * (action - 1))
        response = self.client.sense()
        
        # Convert observation to state vector
        obs = self.convert_observation(response)
        
        limitL = response['limitL']
        limitR = response['limitR']
        
        # Check termination conditions
        terminated = limitL or limitR
        
        # Calculate reward
        reward = self._calculate_reward(obs, action - 1, terminated)
        
        
        
        # Check truncation
        self.step_count += 1
        truncated = self.step_count >= self.max_episode_steps
        
        # Update GUI and episode data
        self._update_gui_and_data(obs, reward, action, terminated, truncated)
        
        return obs, reward, terminated, truncated, {}

    def _calculate_reward(self, state: np.ndarray, action: int, terminated) -> float:
        """
        New reward function using sin(theta) and cos(theta)
        state: [sin(theta), cos(theta), theta_dot, cart_position, cart_velocity]
        """
        sin_theta, cos_theta, theta_dot, x, x_dot = state
        
        # 1. Angle reward (primary objective)
        # Target orientation: (sin=0, cos=-1)
        # Use cosine similarity between current and target orientation
        target_cos = -1.0
        angle_reward = (1.0 + (cos_theta * target_cos))  # Ranges from 0 (down) to 2 (upright)
        
        # 2. Angular velocity penalty
        # angular_vel_penalty = 0.1 * theta_dot ** 2
        
        # 3. Position penalty (keep cart centered)
        position_penalty = 0.2 * x ** 2
        
        # 4. Velocity penalty (prevent rapid movements)
        # velocity_penalty = 0.05 * x_dot ** 2
        
        # 5. Action penalty (encourage minimal control)
        # action_penalty = 0.01 * abs(action - 1)  # action is 0,1,2 mapped to -1,0,+1
        
        # Combine components
        total_reward = (
            angle_reward 
            + (-1000 if terminated else 0)
            # - angular_vel_penalty 
            - position_penalty 
            # - velocity_penalty 
            # - action_penalty
        )
        
        # Add survival bonus if upright
        if angle_reward > 1.9:  # Close to perfect upright
            total_reward += 0.5
            
        return total_reward

    def _update_gui_and_data(self, obs: np.ndarray, reward: float, 
                            action: np.ndarray, terminated: bool, truncated: bool) -> None:
        # Store episode data
        self.episode_data['timesteps'].append(self.step_count)
        self.episode_data['states'].append(obs)
        self.episode_data['rewards'].append(reward)
        self.episode_data['actions'].append(action)
        
        # Update GUI if available
        if self.gui:
            self.gui.update_plots(
                timestep=self.step_count,
                state=obs,
                reward=reward,
                action=action,
                terminated=terminated,
                truncated=truncated
            )

    def _clear_episode_data(self) -> None:
        for key in self.episode_data:
            self.episode_data[key].clear()

    def render(self) -> None:
        """Optional render method for additional visualization"""
        pass

    def close(self) -> None:
        """Clean up resources"""
        self.client.close()
        