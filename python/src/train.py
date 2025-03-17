import numpy as np
import os
import threading
import time
import signal

from queue import Queue
from environment.sim import InvertedPendulumContinuousControlSim
from environment.physical import InvertedPendulumContinuousControlPhysical
from gui import PendulumVisualizerDPG

from stable_baselines3.sac.policies import MlpPolicy
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback


class StateQueueCallback(BaseCallback):
    """
    Custom callback that saves each environment state to a queue after every step.
    """

    def __init__(self, state_queue, verbose=0):
        super(StateQueueCallback, self).__init__(verbose)
        self.state_queue = state_queue

        self.episode = 0
        self.step = 0
        self.start_time = time.time()
        self.episode_start_time = time.time()
        self.episode_reward = 0
        self.episode_rewards = []

    def _on_step(self) -> bool:
        x = self.locals["new_obs"][0][0]
        x_dot = self.locals["new_obs"][0][1]
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

        self.state_queue.put(
            {
                "type": "training",
                "data": {
                    "episode": f"{self.episode}/???",
                    "total time": f"{time.time() - self.start_time:.2f}s",
                    "episode time": f"{time.time() - self.episode_start_time:.2f}s",
                    "step": f"{self.step}/???",
                },
            }
        )
     
        self.episode_reward += self.locals["rewards"][0]

        self.step += 1
        if self.locals["dones"][0]:
            self.episode += 1
            self.episode_start_time = time.time()
            self.step = 0
            self.episode_rewards.append(self.episode_reward)
            episode_reward = self.episode_reward
            self.episode_reward = 0
            avg_score = np.mean(self.episode_rewards[-100:])

            self.state_queue.put(
                {
                    "type": "episode",
                    "data": {
                        "reward": episode_reward,
                        "average_reward": avg_score,
                    },
                }
            )

        return True


def train(environment_class, data_queue, signal_queue):
    env = environment_class()

    agent = SAC(
        MlpPolicy,
        env,
        learning_rate=0.0003,         # Return to default as updates are less frequent
    buffer_size=500000,           # Keep larger buffer
    learning_starts=3,            # Changed to number of episodes (not steps)
    batch_size=1024,               # Increase batch size for episode-based training
    tau=0.005,                    # Default still good
    gamma=0.999,                  # Keep higher gamma for 10ms period
    train_freq=(1, "episode"),    # Train once per episode
    gradient_steps=200,
    ent_coef='auto',              # Auto-tuning still recommended
    target_update_interval=1,     # Update targets every episode
    policy_kwargs=dict(
        net_arch=dict(
            pi=[512, 512],
            qf=[512, 512]
        )
    ),
    verbose=1
    )

    agent.learn(total_timesteps=1000000, callback=StateQueueCallback(data_queue))


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
