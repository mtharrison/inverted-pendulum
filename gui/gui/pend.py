import pygame
import math
import serial
import json
from threading import Lock
from time import perf_counter

class PendulumVisualizer:
    def __init__(self):
        pygame.init()
        self.width, self.height = 1200, 1200
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Pendulum Visualizer")
        
        # Configuration
        self.serial_port = '/dev/cu.usbmodem2101'
        self.baudrate = 115200
        self.update_interval = 0.016  # ~60 Hz
        self.velocity_scale = 40  # Adjust this based on expected velocity range
        
        # Chart parameters
        self.angle_chart_rect = pygame.Rect(100, 700, 1000, 200)
        self.velocity_chart_rect = pygame.Rect(100, 950, 1000, 200)
        self.angle_history = []
        self.velocity_history = []
        
        # State
        self.angle = 0.0
        self.angular_velocity = 0.0
        self.limitL = False
        self.limitR = False
        self.running = True
        self.serial_lock = Lock()
        
        # Initialize serial connection
        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)

    def continuous_angle(self, angle):
        """Convert angle to continuous range [0, 2π)"""
        return angle % (2 * math.pi)

    def fetch_serial_data(self):
        """Read data from serial port and update state"""
        try:
            with self.serial_lock:
                # Request observation data
                self.ser.write(json.dumps([0, 0]).encode() + b'\n')
                response = self.ser.read_until(b'\n').decode().strip()
                
            if response:
                data = json.loads(response)
                if data.get('status') == 'OK':
                    # Update histories before trimming
                    self.angle_history.append(self.continuous_angle(data.get('theta', 0)))
                    self.velocity_history.append(data.get('angular_velocity', 0))
                    
                    # Trim histories to chart widths
                    self.angle_history = self.angle_history[-self.angle_chart_rect.width:]
                    self.velocity_history = self.velocity_history[-self.velocity_chart_rect.width:]
                    
                    return {
                        'angle': data.get('theta', 0),
                        'velocity': data.get('angular_velocity', 0),
                        'limitL': data.get('limitL', False),
                        'limitR': data.get('limitR', False)
                    }
        except (json.JSONDecodeError, serial.SerialException) as e:
            print(f"Serial error: {e}")
        return None

    def handle_input(self):
        """Process keyboard and window events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                
            if event.type == pygame.KEYDOWN:
                step = 0
                if event.key == pygame.K_LEFT:
                    step = -1
                elif event.key == pygame.K_RIGHT:
                    step = 1
                
                if step != 0:
                    try:
                        with self.serial_lock:
                            self.ser.write(json.dumps([0, 1, step]).encode() + b'\n')
                    except serial.SerialException as e:
                        print(f"Write error: {e}")

    def draw_limit_switches(self):
        """Draw limit switch indicators"""
        self.draw_limit((100, 100), self.limitL, "LEFT")
        self.draw_limit((1100, 100), self.limitR, "RIGHT")

    def draw_limit(self, position, triggered, label):
        print(position , triggered, label)
        """Draw individual limit switch"""
        color = (255, 0, 0) if triggered else (255, 255, 255)
        pygame.draw.circle(self.screen, color, position, 20)
        
        font = pygame.font.SysFont(None, 30)
        state = "TRIGGERED" if triggered else ""
        text = font.render(f"{label} {state}", True, (255, 255, 255))
        text_rect = text.get_rect(center=(position[0], position[1]+40))
        self.screen.blit(text, text_rect)

    def draw_angle_chart(self):
        """Render angle history chart"""
        # Chart background
        pygame.draw.rect(self.screen, (0, 0, 0), self.angle_chart_rect)
        pygame.draw.rect(self.screen, (255, 255, 255), self.angle_chart_rect, 1)
        
        # Zero line
        zero_y = self.angle_chart_rect.centery
        pygame.draw.line(self.screen, (255, 255, 255), 
                        (self.angle_chart_rect.left, zero_y),
                        (self.angle_chart_rect.right, zero_y), 1)
        
        # Plot angle history as sine wave
        if len(self.angle_history) > 1:
            points = []
            max_amplitude = self.angle_chart_rect.height // 2 - 5
            
            for i, angle in enumerate(self.angle_history):
                x = self.angle_chart_rect.x + i
                y = zero_y + math.sin(angle) * max_amplitude
                points.append((x, y))
            
            pygame.draw.lines(self.screen, (255, 0, 0), False, points, 2)
        
        # Chart title
        font = pygame.font.SysFont(None, 24)
        text = font.render(f"Angle: {self.continuous_angle(self.angle):.2f} rad", 
                         True, (255, 255, 255))
        text_rect = text.get_rect(midbottom=(self.angle_chart_rect.centerx, 
                                           self.angle_chart_rect.top - 5))
        self.screen.blit(text, text_rect)

    def draw_velocity_chart(self):
        """Render angular velocity history chart"""
        # Chart background
        pygame.draw.rect(self.screen, (0, 0, 0), self.velocity_chart_rect)
        pygame.draw.rect(self.screen, (255, 255, 255), self.velocity_chart_rect, 1)
        
        # Zero line
        zero_y = self.velocity_chart_rect.centery
        pygame.draw.line(self.screen, (255, 255, 255), 
                        (self.velocity_chart_rect.left, zero_y),
                        (self.velocity_chart_rect.right, zero_y), 1)
        
        # Plot velocity history
        if len(self.velocity_history) > 1:
            points = []
            scale_factor = self.velocity_chart_rect.height / self.velocity_scale
            
            for i, velocity in enumerate(self.velocity_history):
                x = self.velocity_chart_rect.x + i
                y = zero_y - velocity * scale_factor  # Negative because pygame y increases downward
                points.append((x, y))
            
            pygame.draw.lines(self.screen, (0, 255, 0), False, points, 2)
        
        # Chart title
        font = pygame.font.SysFont(None, 24)
        text = font.render(f"Angular Velocity: {self.angular_velocity:.2f} rad/s", 
                         True, (255, 255, 255))
        text_rect = text.get_rect(midbottom=(self.velocity_chart_rect.centerx, 
                                           self.velocity_chart_rect.top - 5))
        self.screen.blit(text, text_rect)

    def draw_pendulum(self):
        """Render pendulum visualization"""
        pivot = (self.width//2, 200)
        rod_length = 400
        bob_x = pivot[0] + rod_length * math.sin(-self.angle)
        bob_y = pivot[1] + rod_length * math.cos(self.angle)
        
        pygame.draw.line(self.screen, (255, 255, 255), pivot, (bob_x, bob_y), 5)
        pygame.draw.circle(self.screen, (255, 255, 255), (int(bob_x), int(bob_y)), 60)

    def update_display(self):
        """Update all visual elements"""
        self.screen.fill((0, 0, 0))
        self.draw_pendulum()
        self.draw_limit_switches()
        self.draw_angle_chart()
        self.draw_velocity_chart()
        pygame.display.flip()

    def run(self):
        """Main application loop"""
        last_update = perf_counter()
        
        while self.running:
            self.handle_input()
            
            # Update at fixed intervals
            now = perf_counter()
            if now - last_update >= self.update_interval:
                data = self.fetch_serial_data()
                if data:
                    self.angle = data['angle']
                    self.angular_velocity = data['velocity']
                    self.limitL = data['limitL']
                    self.limitR = data['limitR']
                
                self.update_display()
                last_update = now
            
            pygame.time.wait(1)
        
        self.ser.close()
        pygame.quit()

if __name__ == "__main__":
    visualizer = PendulumVisualizer()
    visualizer.run()