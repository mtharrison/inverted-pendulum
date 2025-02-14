import argparse
import numpy as np
import torch
import os
import random
import math
import wandb
import threading
import time

from queue import Queue
from environment.sim import InvertedPendulumContinuousControlSim
from agent.sac import SACAgent, ReplayBuffer
from gui import PendulumVisualizerDPG


def train(data_queue):
    # Hyperparameters
    state_dim = 5
    action_dim = 1
    max_episodes = 1000
    max_steps = 1000
    batch_size = 256
    buffer_size = int(1e6)
    gamma = 0.99
    tau = 0.005
    alpha = 0.2
    lr = 3e-4

    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--resume", type=str, help="Path to checkpoint to resume from")
    parser.add_argument("--use_wandb", action="store_true", help="Use Weights & Biases")
    args = parser.parse_args()

    device = "mps" if torch.mps.is_available() else "cpu"

    # Initialize W&B
    if args.use_wandb:
        wandb.init(
            project="cartpole-sac",
            config={
                "gamma": gamma,
                "tau": tau,
                "lr": lr,
                "batch_size": batch_size,
                "max_episodes": max_episodes,
            },
            monitor_gym=True,
        )

    # Initialize environment and agent
    env = InvertedPendulumContinuousControlSim(render_mode="human")
    agent = SACAgent(state_dim, action_dim, device, gamma, tau, alpha, lr)

    # Resume training if specified
    start_episode = 0
    best_avg_score = -np.inf
    if args.resume:
        agent.load_checkpoint(args.resume, device)
        start_episode = int(args.resume.split("_")[-1].split(".")[0]) + 1
        print(f"Resuming training from episode {start_episode}")

    buffer = ReplayBuffer(buffer_size)
    scores = []
    avg_scores = []
    start_time = time.time()

    for episode in range(start_episode, max_episodes):
        state, _ = env.reset()
        episode_reward = 0
        episode_losses = []

        before_step = time.time()
        for step in range(max_steps):
            episode_start_time = time.time()
            action = agent.select_action(state)

            next_state, reward, terminated, truncated, _ = env.step(action)

            # env.render()

            current_position, velocity, _, _, angular_velocity = next_state
            data_queue.put(
                {
                    "type": "observation",
                    "data": {
                        "theta": env.state[2],
                        "angular_velocity": angular_velocity,
                        "current_position": current_position,
                        "target_position": current_position,
                        "velocity": velocity,
                        "limitL": terminated,
                        "limitR": terminated,
                        "enabled": True,
                        "resetting": False,
                        "extent": env.x_threshold,
                    },
                }
            )
            done = terminated or truncated

            buffer.add(state, action, reward, next_state, done)
            episode_reward += reward

            before_step = time.time()

            if len(buffer.storage) > batch_size:
                losses = agent.update_parameters(buffer, batch_size)
                episode_losses.append(losses)

            print(f"Step time: {time.time() - before_step:.2f}s")

            state = next_state

            data_queue.put(
                {
                    "type": "training",
                    "data": {
                        "episode": f"{episode}/{max_episodes}",
                        "total time": f"{time.time() - start_time:.2f}s",
                        "episode time": f"{time.time() - episode_start_time:.2f}s",
                    },
                }
            )

            if done:
                break

        # Calculate average losses for the episode
        if episode_losses:
            avg_losses = np.mean(episode_losses, axis=0)
            critic1_loss, critic2_loss, actor_loss, alpha_loss = avg_losses
        else:
            critic1_loss = critic2_loss = actor_loss = alpha_loss = 0.0

        # Update scores and best model
        scores.append(episode_reward)
        avg_score = np.mean(scores[-100:])
        avg_scores.append(avg_score)

        data_queue.put(
            {
                "type": "episode",
                "data": {
                    "reward": episode_reward,
                    "average_reward": avg_score,
                },
            }
        )

        # Save best model
        if avg_score > best_avg_score:
            best_avg_score = avg_score
            filename = f"best_model_{episode}.pth.tar"
            path = agent.save_checkpoint(filename)
            if args.use_wandb:
                wandb.save(path)
                artifact = wandb.Artifact("best-model", type="model")
                artifact.add_file(path)
                wandb.log_artifact(artifact)

        # Save checkpoint periodically
        if episode % 100 == 0 and episode > 0:
            filename = f"checkpoint_{episode}.pth.tar"
            agent.save_checkpoint(filename)
            if args.use_wandb:
                wandb.save(filename)

        # Log to W&B
        if args.use_wandb:
            wandb.log(
                {
                    "Episode Reward": episode_reward,
                    "Average Reward (100)": avg_score,
                    "Critic 1 Loss": critic1_loss,
                    "Critic 2 Loss": critic2_loss,
                    "Actor Loss": actor_loss,
                    "Alpha Loss": alpha_loss,
                    "Alpha": agent.alpha,
                }
            )

        print(
            f"Episode {episode + 1}, Reward: {episode_reward:.2f}, Avg Reward: {avg_score:.2f}"
        )

        if avg_score >= 900:
            print("Environment solved!")
            if args.use_wandb:
                wandb.finish()
            break

    env.close()
    if args.use_wandb:
        wandb.finish()


def main():
    data_queue = Queue()
    # train(data_queue)
    # return

    if os.getenv("DEV", False):
        thrain_thread = threading.Thread(target=train, args=(data_queue,))
        thrain_thread.start()

        visualizer = PendulumVisualizerDPG(data_queue=data_queue)
        visualizer.run()

        thrain_thread.join()
    else:
        # port = os.getenv("SERIAL_PORT", "/dev/cu.usbmodem2101")
        # thrain_thread = threading.Thread(
        #     target=training_thread, args=(port, data_queue)
        # )
        # thrain_thread.start()

        # visualizer = PendulumVisualizerDPG(data_queue=data_queue)
        # visualizer.run()

        # thrain_thread.join()
        pass


if __name__ == "__main__":
    main()
