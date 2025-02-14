from gymnasium import spaces, Env
from gymnasium.utils import seeding
import math
import numpy as np
import logging
import pygame

logger = logging.getLogger(__name__)


class InvertedPendulumContinuousControlSim(Env):
    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": 50,
    }

    def __init__(self, render_mode="human"):
        self.g = 9.82  # gravity
        self.m_c = 0.5  # cart mass
        self.m_p = 0.1  # pendulum mass
        self.total_m = self.m_p + self.m_c
        self.l = 0.25  # pole's length
        self.m_p_l = self.m_p * self.l
        self.force_mag = 10.0
        self.dt = 0.001  # seconds between state updates
        self.b = 1.0  # friction coefficient

        self.t = 0  # timestep
        self.t_limit = 1000

        # Thresholds
        self.theta_threshold_radians = 12 * 2 * math.pi / 360
        self.x_threshold = 0.5

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

        # Rendering
        self.render_mode = render_mode
        self.screen = None
        self.clock = None
        self.screen_dim = 600
        self.scale = self.screen_dim / 5  # 5 meters wide
        self.carty = self.screen_dim // 2  # Cart vertical position

        # Initialize state
        self.np_random, _ = seeding.np_random(None)
        self.state = None

        self.value_surface = None
        self.value_array = np.zeros((100, 100))  # Resolution of value function viz
        self.min_value = -10
        self.max_value = 10

    def step(self, action):
        action = np.clip(action, -1.0, 1.0)[0]
        action *= self.force_mag

        state = self.state
        x, x_dot, theta, theta_dot = state

        s = math.sin(theta)
        c = math.cos(theta)

        # Physics calculations (CORRECTED TORQUE SIGN)
        xdot_update = (
            -2 * self.m_p_l * (theta_dot**2) * s
            + 3 * self.m_p * self.g * s * c
            + 4 * action
            - 4 * self.b * x_dot
        ) / (4 * self.total_m - 3 * self.m_p * c**2)

        thetadot_update = (
            -3 * self.m_p_l * (theta_dot**2) * s * c
            + 6 * self.total_m * self.g * s
            - 6 * (action - self.b * x_dot) * c
        ) / (4 * self.l * self.total_m - 3 * self.m_p_l * c**2)  # Changed sign here

        # Update state
        x += x_dot * self.dt
        theta += theta_dot * self.dt
        x_dot += xdot_update * self.dt
        theta_dot += thetadot_update * self.dt
        self.state = (x, x_dot, theta, theta_dot)

        # Termination conditions
        terminated = abs(x) > self.x_threshold
        truncated = self.t >= self.t_limit
        self.t += 1

        # Reward calculation
        reward_theta = (np.cos(theta) + 1.0) / 2.0
        reward_x = np.cos((x / self.x_threshold) * (np.pi / 2.0))
        reward = reward_theta * reward_x
        if terminated:
            reward = -100.0

        obs = np.array([x, x_dot, np.cos(theta), np.sin(theta), theta_dot])

        return obs, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.state = np.array([0, 0, np.pi, 0])
        self.t = 0
        x, x_dot, theta, theta_dot = self.state
        obs = np.array([x, x_dot, np.cos(theta), np.sin(theta), theta_dot])
        return obs, {}

    def render(self):
        screen_width = self.screen_dim
        screen_height = self.screen_dim
        cart_width = 40
        cart_height = 20

        if self.screen is None:
            pygame.init()
            if self.render_mode == "human":
                pygame.display.init()
                self.screen = pygame.display.set_mode((screen_width, screen_height))
            else:
                self.screen = pygame.Surface((screen_width, screen_height))
            self.clock = pygame.time.Clock()

        self.screen.fill((255, 255, 255))  # White background

        if self.value_surface is not None:
            # Draw in top-right corner
            value_rect = self.value_surface.get_rect()
            value_rect.topright = (self.screen_dim - 10, 10)

            # Draw border
            pygame.draw.rect(self.screen, (0, 0, 0), value_rect.inflate(4, 4), 1)

            # Draw value function
            self.screen.blit(self.value_surface, value_rect)

            # Draw axes labels
            font = pygame.font.Font(None, 24)
            x_label = font.render("Cart Position", True, (0, 0, 0))
            theta_label = font.render("Pole Angle", True, (0, 0, 0))

            # Position labels
            self.screen.blit(
                x_label,
                (value_rect.centerx - x_label.get_width() // 2, value_rect.bottom + 5),
            )
            # Rotate theta label
            theta_label = pygame.transform.rotate(theta_label, 90)
            self.screen.blit(
                theta_label,
                (
                    value_rect.left - 25,
                    value_rect.centery - theta_label.get_width() // 2,
                ),
            )

            # Draw colorbar
            colorbar_height = value_rect.height
            colorbar_width = 20
            colorbar_x = value_rect.right + 20
            colorbar_y = value_rect.top

            for i in range(colorbar_height):
                value = 1 - (i / colorbar_height)
                r = int(255 * value)
                b = int(255 * (1 - value))
                pygame.draw.line(
                    self.screen,
                    (r, 0, b),
                    (colorbar_x, colorbar_y + i),
                    (colorbar_x + colorbar_width, colorbar_y + i),
                )

            # Draw colorbar labels
            max_label = font.render(f"{self.max_value:.1f}", True, (0, 0, 0))
            min_label = font.render(f"{self.min_value:.1f}", True, (0, 0, 0))
            self.screen.blit(max_label, (colorbar_x + colorbar_width + 5, colorbar_y))
            self.screen.blit(
                min_label,
                (
                    colorbar_x + colorbar_width + 5,
                    colorbar_y + colorbar_height - min_label.get_height(),
                ),
            )

        # Get current state
        x_pos, _, theta, _ = self.state

        # Calculate positions
        cartx = x_pos * self.scale + screen_width / 2.0
        pole_length = self.l * self.scale

        # Draw track
        pygame.draw.line(
            self.screen,
            (0, 0, 0),
            (
                screen_width // 2 - self.x_threshold * self.scale,
                self.carty + cart_height,
            ),
            (
                screen_width // 2 + self.x_threshold * self.scale,
                self.carty + cart_height,
            ),
            3,
        )

        # Draw cart
        cart_rect = pygame.Rect(
            cartx - cart_width / 2,
            self.carty - cart_height / 2,
            cart_width,
            cart_height,
        )
        pygame.draw.rect(self.screen, (255, 0, 0), cart_rect)

        # Draw wheels
        wheel_radius = cart_height // 4
        pygame.draw.circle(
            self.screen,
            (0, 0, 0),
            (int(cartx - cart_width / 2), int(self.carty + cart_height / 2)),
            wheel_radius,
        )
        pygame.draw.circle(
            self.screen,
            (0, 0, 0),
            (int(cartx + cart_width / 2), int(self.carty + cart_height / 2)),
            wheel_radius,
        )

        # Draw pole
        pole_end_x = cartx + pole_length * math.sin(theta)
        pole_end_y = self.carty - pole_length * math.cos(theta)
        pygame.draw.line(
            self.screen,
            (0, 0, 255),
            (cartx, self.carty),
            (pole_end_x, pole_end_y),
            6,  # Pole width
        )

        # Draw pole bob
        pygame.draw.circle(
            self.screen,
            (0, 0, 0),
            (int(pole_end_x), int(pole_end_y)),
            6,  # Bob radius
        )

        if self.render_mode == "human":
            pygame.event.pump()
            pygame.display.flip()
            self.clock.tick(self.metadata["render_fps"])
        else:
            return np.transpose(
                np.array(pygame.surfarray.pixels3d(self.screen)), axes=(1, 0, 2)
            )

    def close(self):
        if self.screen is not None:
            self.screen = None
