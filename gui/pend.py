import pygame
import math
import serial
import msgpack
import time
import json
i = 0
# Initialize Pygame
pygame.init()

# Set up the display
width, height = 1200, 1200
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Pendulum Visualizer with Rotation Chart")

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

# Pendulum parameters
pivot = (width // 2, 100)
rod_length = 200 * 2
bob_radius = 30 * 2

# Chart parameters
chart_rect = pygame.Rect(600-350, 400 * 2, 700, 150 * 2)  # x, y, width, height
rotation_history = []
max_history_length = chart_rect.width  # Match chart width

def render(rotation):
    global rotation_history
    
    # Clamp rotation and update history
    rotation = rotation if rotation <= 1200 else rotation - 2400
    rotation_history.append(rotation)
    
    # Keep history bounded to chart width
    if len(rotation_history) > max_history_length:
        rotation_history = rotation_history[-max_history_length:]
    
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            return False
    
    # Calculate pendulum position
    angle = -rotation * (math.pi / 1200)
    bob_x = pivot[0] + rod_length * math.sin(angle)
    bob_y = pivot[1] + rod_length * math.cos(angle)
    
    # Draw everything
    screen.fill(BLACK)
    
    # Draw pendulum
    pygame.draw.line(screen, WHITE, pivot, (bob_x, bob_y), 5)
    pygame.draw.circle(screen, WHITE, (int(bob_x), int(bob_y)), bob_radius)
    
    # Draw chart
    draw_chart(rotation)
    
    pygame.display.flip()
    return True

def draw_chart(rotation):
    # Chart background
    pygame.draw.rect(screen, BLACK, chart_rect)
    pygame.draw.rect(screen, WHITE, chart_rect, 1)
    
    # Draw zero line
    zero_y = chart_rect.y + chart_rect.height * 0.5
    pygame.draw.line(screen, WHITE, (chart_rect.left, zero_y), 
                    (chart_rect.right, zero_y), 1)
    
    # Draw rotation values if we have data
    if len(rotation_history) > 1:
        points = []
        for i, rotation in enumerate(rotation_history):
            x = chart_rect.x + i
            y = chart_rect.y + chart_rect.height * (1200 - rotation) / 2400
            points.append((x, y))
        
        pygame.draw.lines(screen, RED, False, points, 2)
    
    # Draw current rotation text
    font = pygame.font.Font(None, 24)
    text = font.render(f"Current Rotation: {rotation}", True, WHITE)
    screen.blit(text, (chart_rect.x + 10, chart_rect.y + 10))

# Example usage
if __name__ == "__main__":
    clock = pygame.time.Clock()
    running = True
    angle = 0
    
    with serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=3) as ser:
        while running:
            ser.write(json.dumps([i, "observe"]).encode())
            out = ser.read_until(b']')
            print(out)
            angle = json.loads(out)[4]

            # Update display and check if window was closed
            if not render(angle):
                running = False

            clock.tick(30)
                
    
    pygame.quit()