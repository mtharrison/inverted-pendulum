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

        self.client = SerialCommunicator(port="/dev/cu.usbmodem2201")

        self.t = 0  # timestep
        self.t_limit = 5000

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
        self.state = (0, 0, math.pi, 0)

    def convert_observation(self, response: dict) -> np.ndarray:
        extent = response["extent"] if response["extent"] > 0 else 1000
        self.x_threshold = extent
        theta = response["theta"]
        theta_dot = response["angular_velocity"] / 10
        x = response["current_position"] / self.x_threshold
        x_dot = response["velocity"] / (self.x_threshold * 10)

        return np.array(
            [x, x_dot, np.cos(theta), np.sin(theta), theta_dot], dtype=np.float32
        )

    def reset(
        self, seed: Optional[int] = None, options: Optional[dict] = None
    ) -> Tuple[np.ndarray, dict]:
        self.state = np.array([0, 0, np.pi, 0], dtype=np.float32)
        self.t = 0

        self.client.reset()

        # wait until resetting=false
        while True:
            response = self.client.sense()
            print(response)
            if (
                response is not None
                and not response["resetting"]
                and response["angular_velocity"] < 1
                and math.cos(response["theta"]) < -0.9
            ):
                break
            time.sleep(1)

        time.sleep(2)

        obs = self.convert_observation(self.client.sense())

        return obs, {}

    def step(self, action):
        action = np.clip(action, -1.0, 1.0)[0]
        response = self.client.move(action.item())

        if response is None:
            return self.last_step_return

        self.state[2] = response["theta"]

        # Convert observation to state vector
        obs = self.convert_observation(response)

        limitL = response["limitL"]
        limitR = response["limitR"]

        # Check truncation
        truncated = self.t >= self.t_limit

        # Termination conditions
        terminated = (limitL or limitR) or abs(obs[4]) > 16.0
        truncated = bool(self.t >= self.t_limit)
        self.t += 1

        reward = 0.5 * (1 + obs[2])
        # Update GUI and episode data
        self.last_step_return = (obs, reward, terminated, truncated, {})

        return obs, reward, terminated, truncated, {}

    def __del__(self) -> None:
        self.client.close()
