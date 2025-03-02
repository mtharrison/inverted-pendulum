import argparse
import numpy as np
import torch
import os
import wandb
import threading
import time
import signal

from queue import Queue
from environment.sim import InvertedPendulumContinuousControlSim
from environment.physical import InvertedPendulumContinuousControlPhysical
from agent.sac import SACAgent, ReplayBuffer
from gui import PendulumVisualizerDPG

from stable_baselines3.sac.policies import MlpPolicy
from stable_baselines3 import SAC
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import BaseCallback


class StateQueueCallback(BaseCallback):
    """
    Custom callback that saves each environment state to a queue after every step.
    """

    def __init__(self, state_queue, verbose=0):
        super(StateQueueCallback, self).__init__(verbose)
        self.state_queue = state_queue

    def _on_step(self) -> bool:
        # x, x_dot, cos_theta, sin_theta, theta_dot = self.locals["new_obs"]
        x = self.locals["new_obs"][0][0]
        x_dot = self.locals["new_obs"][0][1]
        cos_theta = self.locals["new_obs"][0][2]
        sin_theta = self.locals["new_obs"][0][3]
        theta_dot = self.locals["new_obs"][0][4]

        current_state = self.training_env.envs[0].unwrapped.state
        self.state_queue.put(
            {
                "type": "observation",
                "data": {
                    "theta": current_state[2],
                    "angular_velocity": theta_dot,
                    "current_position": x,
                    "target_position": x,
                    "velocity": x_dot,
                    "limitL": False,
                    "limitR": False,
                    "enabled": True,
                    "resetting": False,
                    "extent": 1.0,
                },
            }
        )
        return True


def train(environment_class, data_queue, signal_queue):
    check_env(environment_class())

    # Set up the environment
    env = environment_class()

    # Set up the agent
    agent = SAC(
        MlpPolicy,
        env,
        verbose=1,
        tensorboard_log="./tensorboard_logs",
    )

    # Train the agent
    agent.learn(total_timesteps=100000, callback=StateQueueCallback(data_queue))


def main():
    data_queue = Queue()
    signal_queue = Queue()

    if os.getenv("DEV", False) == "true":
        environment_class = InvertedPendulumContinuousControlSim
    else:
        environment_class = InvertedPendulumContinuousControlPhysical

    thrain_thread = threading.Thread(
        target=train,
        args=(
            environment_class,
            data_queue,
            signal_queue,
        ),
    )
    thrain_thread.start()

    visualizer = PendulumVisualizerDPG(data_queue=data_queue)

    # handle ctrl+c
    def signal_handler(sig, frame):
        signal_queue.put({"type": "stop"})

    signal.signal(signal.SIGINT, signal_handler)

    visualizer.run()

    signal_queue.put({"type": "stop"})

    thrain_thread.join()


if __name__ == "__main__":
    main()
