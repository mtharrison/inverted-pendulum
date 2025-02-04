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
chart_rect = pygame.Rect(100, 400 * 2, 450, 150 * 2)  # x, y, width, height
chart_rect2 = pygame.Rect(650, 400 * 2, 450, 150 * 2)  # x, y, width, height
angle_history = []
angular_velocity_history = []
max_history_length = chart_rect.width  # Match chart width

def render(angle, angular_velocity, limitL, limitR):
    global angle_history, angular_velocity_history
    
    angle_history.append(angle)
    angular_velocity_history.append(angular_velocity)
    
    # Keep history bounded to chart width
    if len(angle_history) > max_history_length:
        angle_history = angle_history[-max_history_length:]
    if len(angular_velocity_history) > max_history_length:
        angular_velocity_history = angular_velocity_history[-max_history_length:]
    
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            return False
    
    # Calculate pendulum position
    bob_x = pivot[0] + rod_length * math.sin(-angle)
    bob_y = pivot[1] + rod_length * math.cos(angle)
    
    # Draw everything
    screen.fill(BLACK)
    
    # Draw pendulum
    pygame.draw.line(screen, WHITE, pivot, (bob_x, bob_y), 5)
    pygame.draw.circle(screen, WHITE, (int(bob_x), int(bob_y)), bob_radius)

    # Draw limits
    font = pygame.font.Font(None, 24)
    
    pygame.draw.circle(screen, WHITE if limitL is 0 else RED, (100, 100), 20)
    text_left = font.render(f"{"Limit Left" if not limitL else "TRIGGERED"}", True, WHITE)
    screen.blit(text_left, (100 - text_left.get_rect().width / 2, 100 + 40))

    text_right = font.render(f"{"Limit Right" if not limitR else "TRIGGERED"}", True, WHITE)
    pygame.draw.circle(screen, WHITE if limitR is 0 else RED, (1100, 100), 20)
    screen.blit(text_right, (1100 - text_right.get_rect().width / 2, 100 + 40))
    
    # Draw chart
    draw_chart(angle, angular_velocity)
    
    pygame.display.flip()
    return True

def continuous_angle_to_plus_minus(angle):
    mod_angle = angle % (2*math.pi)
    return mod_angle if mod_angle <= math.pi else (mod_angle - 2 * math.pi )

def draw_chart(angle, angular_velocity):
    # Chart background
    pygame.draw.rect(screen, BLACK, chart_rect)
    pygame.draw.rect(screen, WHITE, chart_rect, 1)
    
    # Draw zero line
    zero_y = chart_rect.y + chart_rect.height * 0.5
    pygame.draw.line(screen, WHITE, (chart_rect.left, zero_y), 
                    (chart_rect.right, zero_y), 1)
    
    # Draw rotation values if we have data
    if len(angle_history) > 1:
        points = []
        for i, angle in enumerate(angle_history):
            
            y0 = chart_rect.y + chart_rect.height / 2
            x = chart_rect.x + i
            y = y0 + (math.sin(angle) * chart_rect.height / 2)
            
            points.append((x,y))
        
        pygame.draw.lines(screen, RED, False, points, 2)

    # Chart background
    pygame.draw.rect(screen, BLACK, chart_rect2)
    pygame.draw.rect(screen, WHITE, chart_rect2, 1)
    
    # Draw zero line
    zero_y_2 = chart_rect2.y + chart_rect2.height * 0.5
    pygame.draw.line(screen, WHITE, (chart_rect2.left, zero_y_2), 
                    (chart_rect2.right, zero_y_2), 1)
    
    # Draw rotation values if we have data
    if len(angular_velocity_history) > 1:
        points2 = []
        for i, velocity in enumerate(angular_velocity_history):
            
            y0 = chart_rect2.y + chart_rect2.height / 2
            x = chart_rect2.x + i
            y = y0 + (velocity ) * (chart_rect2.height / 40)
            
            points2.append((x,y))
        
        pygame.draw.lines(screen, RED, False, points2, 2)
    
    # Draw current rotation text
    font = pygame.font.Font(None, 24)
    text = font.render(f"Current Angle: {continuous_angle_to_plus_minus(angle):.3f}", True, WHITE)
    screen.blit(text, (chart_rect.x, chart_rect.y - 25))

    text2 = font.render(f"Current angular velocity: {angular_velocity :.3f}", True, WHITE)
    screen.blit(text2, (650, chart_rect.y - 25))

# Example usage
if __name__ == "__main__":
    clock = pygame.time.Clock()
    running = True
    prev_time = time.time()
    pygame.key.set_repeat(50, 1)
    with serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=0.1) as ser:
        while running:
            t = time.time()
            if (t - prev_time > 16/1000):
                ser.write((json.dumps([i, 0]) + '\n').encode())
                out = ser.read_until(b']')
                print(out)
                try:
                    parsed = json.loads(out) 
                    angle = parsed[4]  # a continous angle
                    angular_velocity = parsed[5]
                    limitL = parsed[6]
                    limitR = parsed[7]

                    # Update display and check if window was closed
                    if not render(angle, angular_velocity, limitL, limitR):
                        running = False
                except:
                    pass
                prev_time = time.time()

                

            events = pygame.event.get()
            for event in events:
                if event.type == pygame.KEYDOWN:
                    move = 0
                    if event.key == pygame.K_LEFT:
                        move = -1
                    if event.key == pygame.K_RIGHT:
                        move = 1

                    ser.write((json.dumps([i, 1, move]) + '\n').encode())
                    print('writing', move)

                    ser.write((json.dumps([i, 0]) + '\n').encode())
                    out = ser.read_until(b']')
                    print(out)

    pygame.quit()