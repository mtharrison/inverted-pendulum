import gymnasium as gym
from agent import DQNAgent, DQN, TrainingLogger, ReplayBuffer
from gym_environment import InvertedPendulumEnv
from serial_client import SerialCommunicator
import numpy as np


def train(env: gym.Env, agent: DQNAgent, logger: TrainingLogger,
          num_episodes: int = 1000, batch_size: int = 64,
          target_update: int = 10, save_path: str = "best_model.pth"):
    buffer = ReplayBuffer(capacity=10000)
    best_reward = -np.inf
    
    for episode in range(1, num_episodes + 1):
        state, _ = env.reset()
        episode_reward = 0
        episode_length = 0
        done = False
        
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

# Usage example
if __name__ == "__main__":
    # Initialize your environment and GUI
    client = SerialCommunicator("/dev/cu.usbmodem2101")
    env = InvertedPendulumEnv(client)
    
    # Create agent and logger
    agent = DQNAgent(
        state_dim=env.observation_space.shape[0],
        action_dim=env.action_space.n,
        lr=1e-3,
        gamma=0.99,
        epsilon_decay=0.995
    )
    
    logger = TrainingLogger()
    
    # Train the agent
    train(env, agent, logger, num_episodes=500)