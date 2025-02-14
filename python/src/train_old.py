import gymnasium as gym
from rl_agent import DQNAgent, TrainingLogger, ReplayBuffer
from rl_environment import InvertedPendulumEnv
from serial_client import SerialCommunicator
from serial_virtual import VirtualSerialPair
from serial_mock import MockSerialEndpoint
from gui import PendulumVisualizerDPG
import os

import numpy as np
import random
import math
import threading
from queue import Queue
import time


def train(
    env: gym.Env,
    agent: DQNAgent,
    logger: TrainingLogger,
    num_episodes: int = 1000,
    batch_size: int = 64,
    target_update: int = 10,
    save_path: str = "best_model.pth",
    data_queue: Queue = None,
):
    buffer = ReplayBuffer(capacity=10000)
    best_reward = -np.inf
    start_time = time.time()

    for episode in range(1, num_episodes + 1):
        state, _ = env.reset()
        episode_reward = 0
        episode_length = 0
        done = False
        episode_start_time = time.time()

        while not done:
            action = agent.get_action(state)
            next_state, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated

            buffer.push(state, action, next_state, reward, done)

            loss = agent.update_model(buffer, batch_size)
            logger.log_step(reward, loss)

            state = next_state
            episode_reward += reward
            episode_length += 1

            data_queue.put(
                {
                    "type": "training",
                    "data": {
                        "episode": f"{episode}/{num_episodes}",
                        "epsilon": f"{agent.epsilon:.2f}",
                        "total time": f"{time.time() - start_time:.2f}s",
                        "episode time": f"{time.time() - episode_start_time:.2f}s",
                        "steps": episode_length,
                    },
                }
            )

        data_queue.put(
            {
                "type": "episode",
                "data": {
                    "reward": episode_reward,
                    "loss": loss,
                },
            }
        )

        # Update target network
        if episode % target_update == 0:
            agent.update_target_net()

        # Update epsilon
        agent.update_epsilon()

        # Logging
        logger.log_episode(episode_reward, episode_length, agent.epsilon)

        # Save best model
        if episode_reward > best_reward:
            best_reward = episode_reward
            agent.save(save_path)

    return logger


def training_thread(port, data_queue):
    client = SerialCommunicator(port)
    print(client.sense())
    env = InvertedPendulumEnv(client, data_queue=data_queue)
    agent = DQNAgent(
        state_dim=env.observation_space.shape[0],
        action_dim=env.action_space.n,
        lr=1e-3,
        gamma=0.99,
        epsilon_decay=0.995,
    )
    logger = TrainingLogger()
    train(env, agent, logger, num_episodes=500, data_queue=data_queue)


# Usage example
def main():
    data_queue = Queue()

    if os.getenv("DEV", False):

        def on_sense(state, request):
            state["theta"] += 0.01
            state["angular_velocity"] = random.uniform(-1, 1)
            state["current_position"] = math.cos(state["theta"]) * 1000
            state["target_position"] = (math.cos(state["theta"]) * 1000) + 200
            state["velocity"] = random.uniform(-1, 1)
            state["limitL"] = False
            state["limitR"] = False
            state["enabled"] = True if random.uniform(-1, 1) < 0 else False
            state["resetting"] = state["resetting"]
            state["extent"] = 1000

            return {"status": "OK", **state, "id": request["id"]}

        def on_move(state, request):
            state["target"] += request["params"]["distance"]
            return {"status": "OK", "id": request["id"]}

        def on_reset(state, request):
            # state["resetting"] = True
            return {"status": "OK", "id": request["id"], "resetting": True}

        with VirtualSerialPair() as (port1, port2):
            MockSerialEndpoint(
                port=port1, on_sense=on_sense, on_move=on_move, on_reset=on_reset
            )

            thrain_thread = threading.Thread(
                target=training_thread, args=(port2, data_queue)
            )
            thrain_thread.start()

            visualizer = PendulumVisualizerDPG(data_queue=data_queue)
            visualizer.run()

            thrain_thread.join()
    else:
        port = os.getenv("SERIAL_PORT", "/dev/cu.usbmodem2101")
        thrain_thread = threading.Thread(
            target=training_thread, args=(port, data_queue)
        )
        thrain_thread.start()

        visualizer = PendulumVisualizerDPG(data_queue=data_queue)
        visualizer.run()

        thrain_thread.join()


if __name__ == "__main__":
    main()
