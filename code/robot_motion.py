import pygame
import numpy as np

pygame.init()

# Constants
WIDTH, HEIGHT = 800, 600
ROOM_COLOR = (220, 220, 220)  # Light gray background
ROBOT_COLOR = (0, 100, 255)
WHEEL_COLOR = (0, 0, 0)  # Black wheels
WALL_COLOR = (150, 0, 0)  # Dark red walls
START_COLOR = (0, 255, 0)
GOAL_COLOR = (255, 215, 0)  # Gold for goal
SPEED = 1  # Reduced speed
FPS = 30

# Set up the display
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Motion Planning")

# Load robot image
robot_image = pygame.image.load("images/robot_image.jpg").convert_alpha()
robot_image = pygame.transform.scale(robot_image, (30, 20))

# Define walls (as rectangles)
walls = [
    pygame.Rect(100, 100, 600, 10),  # Top wall
    pygame.Rect(100, 100, 10, 400),  # Left wall
    pygame.Rect(100, 490, 600, 10),  # Bottom wall
    pygame.Rect(690, 100, 10, 400),  # Right wall
    # pygame.Rect(300, 200, 200, 10),  # Middle horizontal wall
    pygame.Rect(300, 200, 10, 200),  # Middle vertical wall
    pygame.Rect(500, 350, 10, 100),  # Vertical wall
    pygame.Rect(400, 300, 10, 100),  # Vertical wall
    pygame.Rect(200, 300, 10, 100),  # Vertical wall
    pygame.Rect(200, 450, 200, 10),  # Bottom horizontal wall
    pygame.Rect(450, 150, 10, 100),  # Vertical wall
]

start_pos = np.array([150, 150], dtype=float)
goal_pos = np.array([650, 450], dtype=float)

# Set up font for text
font = pygame.font.SysFont("Arial", 24)

def draw_walls():
    for wall in walls:
        pygame.draw.rect(screen, WALL_COLOR, wall)

def check_collision(robot_pos):
    robot_rect = pygame.Rect(robot_pos[0] - 15, robot_pos[1] - 10, 30, 20)  # Robot dimensions
    for wall in walls:
        if robot_rect.colliderect(wall):
            return True
    return False

def steer_towards(target, current):
    direction = target - current
    distance = np.linalg.norm(direction)
    
    if distance < SPEED:
        return target  # Move directly to target if within speed limit
    else:
        direction /= distance  # Normalize
        return current + direction * SPEED  # Move at defined speed

def draw_robot(pos):
    # Draw the robot as a rounded rectangle (car-like shape)
    # robot_rect = pygame.Rect(pos[0] - 15, pos[1] - 10, 30, 20)
    # pygame.draw.rect(screen, ROBOT_COLOR, robot_rect, border_radius=5)

    # # Draw wheels
    # wheel_radius = 5
    # wheel_offset_x = 10  # Half the width of the robot
    # wheel_offset_y = 15  # Distance from the bottom of the robot
    # pygame.draw.circle(screen, WHEEL_COLOR, (int(pos[0] - wheel_offset_x), int(pos[1] + wheel_offset_y)), wheel_radius)
    # pygame.draw.circle(screen, WHEEL_COLOR, (int(pos[0] + wheel_offset_x), int(pos[1] + wheel_offset_y)), wheel_radius)

    # Draw the robot image
    robot_rect = robot_image.get_rect(center=(pos[0], pos[1]))
    screen.blit(robot_image, robot_rect.topleft)

def draw_labels():
    # Draw start point label
    start_label = font.render("Start Point", True, (0, 0, 0))
    screen.blit(start_label, (start_pos[0] + 10, start_pos[1] - 30))

    # Draw goal point label
    goal_label = font.render("Goal Point", True, (0, 0, 0))
    screen.blit(goal_label, (goal_pos[0] + 10, goal_pos[1] - 30))

def draw_room_background():
    # Draw room floor
    pygame.draw.rect(screen, (240, 240, 240), (0, 0, WIDTH, HEIGHT))

    pygame.draw.rect(screen, (150, 75, 0), (300, 300, 100, 20))  # Table

# Main loop
running = True
path = []
current_pos = start_pos.copy()

# Plan the path before movement
while np.linalg.norm(current_pos - goal_pos) >= 15:
    next_pos = steer_towards(goal_pos, current_pos)

    if check_collision(next_pos):
        # If a collision is detected, adjust the position
        angle = np.arctan2(next_pos[1] - current_pos[1], next_pos[0] - current_pos[0])
        avoidance_vector = np.array([-np.sin(angle), np.cos(angle)]) * SPEED
        next_pos = current_pos + avoidance_vector

    path.append(next_pos.copy())
    current_pos = next_pos

# Main loop for robot movement
robot_pos = start_pos.copy()
clock = pygame.time.Clock()

while running:
    screen.fill(ROOM_COLOR)
    draw_room_background()
    draw_walls()
    
    # Draw start and goal
    pygame.draw.circle(screen, START_COLOR, (int(start_pos[0]), int(start_pos[1])), 5)
    pygame.draw.circle(screen, GOAL_COLOR, (int(goal_pos[0]), int(goal_pos[1])), 5)

    # Draw labels for start and goal points
    draw_labels()

    # Draw the planned path
    if len(path) > 1:
        pygame.draw.lines(screen, (0, 255, 0), False, path, 2)

    # Move the robot towards the goal while avoiding walls
    for _ in range(5):  # More steps to slow down the movement
        if np.linalg.norm(robot_pos - goal_pos) < 15:
            robot_pos = goal_pos  # Snap to goal position
            break

        next_pos = steer_towards(goal_pos, robot_pos)

        if check_collision(next_pos):
            # Adjust position away from the wall if a collision is detected
            angle = np.arctan2(next_pos[1] - robot_pos[1], next_pos[0] - robot_pos[0])
            avoidance_vector = np.array([-np.sin(angle), np.cos(angle)]) * SPEED
            next_pos = robot_pos + avoidance_vector

        robot_pos = next_pos
        draw_robot(robot_pos)
        pygame.display.flip()
        clock.tick(FPS)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

pygame.quit()
