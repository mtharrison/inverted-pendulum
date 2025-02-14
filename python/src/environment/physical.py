import gymnasium as gym
from gymnasium import spaces
from gymnasium.utils import seeding
import numpy as np
from typing import Optional, Tuple
import time
import math
from serial_communication.client import SerialCommunicator


class InvertedPendulumContinuousControlPhysical(gym.Env):
    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": 50,
    }

    def __init__(
        self,
        render_mode: str = "human",
    ):
        super().__init__()

        self.client = SerialCommunicator(port='/dev/cu.usbmodem2201')

        self.t = 0  # timestep
        self.t_limit = 1000

        # Observation and action spaces
        high = np.array(
            [
                np.finfo(np.float32).max,
                np.finfo(np.float32).max,
                np.finfo(np.float32).max,
                np.finfo(np.float32).max,
                np.finfo(np.float32).max,
            ]
        )
        self.action_space = spaces.Box(-1.0, 1.0, shape=(1,))
        self.observation_space = spaces.Box(-high, high)

        self.np_random, _ = seeding.np_random(None)
        self.state = (0,0,math.pi,0)


    def convert_observation(self, response: dict) -> np.ndarray:
        extent = response["extent"] if response["extent"] > 0 else 1000
        self.x_threshold = extent
        theta = response["theta"]
        theta_dot = response["angular_velocity"]
        x = response["current_position"] / self.x_threshold
        x_dot = response["velocity"] / self.x_threshold
        
        return np.array([x, x_dot, np.cos(theta), np.sin(theta), theta_dot])

    def reset(
        self, seed: Optional[int] = None, options: Optional[dict] = None
    ) -> Tuple[np.ndarray, dict]:
        
        self.state = np.array([0, 0, np.pi, 0])
        self.t = 0

        self.client.reset()

        # wait until resetting=false
        while True:
            response = self.client.sense()
            if response is not None and not response["resetting"]:
                break
            time.sleep(1)

        obs = self.convert_observation(self.client.sense())

        return obs, {}

    def step(self, action):
        action = np.clip(action, -1.0, 1.0)[0]
        self.client.move(action.item())
        time.sleep(0.001)
        response = self.client.sense()
        if response is None:
            return self.last_step_return
        
        self.state[2] = response["theta"]
        
        # Convert observation to state vector
        obs = self.convert_observation(response)

        limitL = response["limitL"]
        limitR = response["limitR"]

        # Check termination conditions
        terminated = limitL or limitR

        # Calculate reward
        reward = self._calculate_reward(obs, terminated)

        # Check truncation
        self.t += 1
        truncated = self.t >= self.t_limit

        # Update GUI and episode data
        self.last_step_return = (obs, reward, action, terminated, truncated)

        return obs, reward, terminated, truncated, {}

    def _calculate_reward(self, obs, terminated) -> float:
        #np.array([x, x_dot, np.cos(theta), np.sin(theta), theta_dot])
        
        x, x_dot, cos_theta, sin_theta, theta_dot = obs
        theta = self.state[2]
        
        reward_theta = (cos_theta + 1.0) / 2.0
        reward_x = np.cos(x * (np.pi / 2.0))
        reward_bonus = 0.0
        if np.cos(theta) > 0.999:
            reward_bonus = 0.5
        reward = reward_theta * reward_x + reward_bonus
        if terminated:
            reward = -1.0
            
        return reward

    def __del__(self) -> None:
        self.client.close()
