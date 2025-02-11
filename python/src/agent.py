import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque, namedtuple
import random
from typing import Optional
import gymnasium as gym

# Define experience tuple
Transition = namedtuple('Transition', 
                        ('state', 'action', 'next_state', 'reward', 'done'))

class ReplayBuffer:
    def __init__(self, capacity: int = 10000):
        self.buffer = deque(maxlen=capacity)
    
    def push(self, *args):
        self.buffer.append(Transition(*args))
    
    def sample(self, batch_size: int) -> list:
        return random.sample(self.buffer, batch_size)
    
    def __len__(self) -> int:
        return len(self.buffer)

class DQN(nn.Module):
    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 128):
        super(DQN, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim)
        )
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)

class DQNAgent:
    def __init__(self, state_dim: int, action_dim: int, 
                 lr: float = 1e-3, gamma: float = 0.99,
                 epsilon_max: float = 0.9, epsilon_min: float = 0.05,
                 epsilon_decay: float = 0.995):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.action_dim = action_dim
        self.gamma = gamma
        self.epsilon = epsilon_max
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        
        # Networks
        self.policy_net = DQN(state_dim, action_dim).to(self.device)
        self.target_net = DQN(state_dim, action_dim).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()
        
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=lr)
        self.loss_fn = nn.SmoothL1Loss()  # Huber loss
        
    def get_action(self, state: np.ndarray, training: bool = True) -> int:
        if training and np.random.random() < self.epsilon:
            return np.random.randint(self.action_dim)
        
        state_t = torch.FloatTensor(state).to(self.device).unsqueeze(0)
        with torch.no_grad():
            q_values = self.policy_net(state_t)
        return q_values.argmax(dim=1).item()
    
    def update_epsilon(self):
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
    
    def update_model(self, buffer: ReplayBuffer, batch_size: int):
        if len(buffer) < batch_size:
            return None
        
        transitions = buffer.sample(batch_size)
        batch = Transition(*zip(*transitions))
        
        # Convert to tensors
        state_batch = torch.FloatTensor(np.array(batch.state)).to(self.device)
        action_batch = torch.LongTensor(batch.action).to(self.device).unsqueeze(1)
        reward_batch = torch.FloatTensor(batch.reward).to(self.device)
        next_state_batch = torch.FloatTensor(np.array(batch.next_state)).to(self.device)
        done_batch = torch.FloatTensor(batch.done).to(self.device)
        
        # Compute Q(s_t, a)
        q_values = self.policy_net(state_batch).gather(1, action_batch)
        
        # Compute V(s_{t+1})
        next_q_values = self.target_net(next_state_batch).max(1)[0].detach()
        expected_q_values = reward_batch + (1 - done_batch) * self.gamma * next_q_values
        
        # Compute loss
        loss = self.loss_fn(q_values, expected_q_values.unsqueeze(1))
        
        # Optimize
        self.optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.policy_net.parameters(), 1.0)  # Gradient clipping
        self.optimizer.step()
        
        return loss.item()
    
    def update_target_net(self):
        self.target_net.load_state_dict(self.policy_net.state_dict())
    
    def save(self, path: str):
        torch.save(self.policy_net.state_dict(), path)
    
    def load(self, path: str):
        self.policy_net.load_state_dict(torch.load(path))
        self.target_net.load_state_dict(self.policy_net.state_dict())

class TrainingLogger:
    def __init__(self, log_dir: Optional[str] = None):
        self.episode_rewards = []
        self.episode_lengths = []
        self.episode_losses = []
        self.timestep = 0
        
    def log_step(self, reward: float, loss: Optional[float]):
        self.timestep += 1
        if loss is not None:
            self.episode_losses.append(loss)
    
    def log_episode(self, reward: float, length: int, epsilon: float):
        self.episode_rewards.append(reward)
        self.episode_lengths.append(length)
        
        print(f"Episode {len(self.episode_rewards)}: "
              f"Reward: {reward:.2f}, "
              f"Length: {length}, "
              f"Epsilon: {epsilon:.3f}, "
              f"Avg Loss: {np.mean(self.episode_losses[-100:]):.4f}")

