import pygame
import os
import math

# Constants
WIDTH, HEIGHT = 900, 900
CELL_SIZE = 50
ROWS, COLS = HEIGHT // CELL_SIZE, WIDTH // CELL_SIZE
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Movement: 2D vs 3D with Enhanced Potential Field")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)  # Goal color
RED = (255, 0, 0)    # Start color
BLUE = (0, 0, 255)   # Path color
FPS = 15
ROBOT_WIDTH, ROBOT_HEIGHT = CELL_SIZE, CELL_SIZE

# Load images
RAHALROBOT = pygame.image.load(os.path.join('1.introduction/images', 'robot_image.jpg'))   # --lsa el rotate-- 
RAHAL_SCALED = pygame.transform.scale(RAHALROBOT, (ROBOT_WIDTH, ROBOT_HEIGHT))

robot_rect = pygame.Rect(1 * CELL_SIZE, 1 * CELL_SIZE, ROBOT_WIDTH, ROBOT_HEIGHT)
goal_point = None
movement_type = "Discrete"  # Start in Discrete movement mode
moving = False  # Control robot movement with space key
moving_index = 0  # Index for the current position in the path
path = []  # Initialize path

pygame.font.init()
font = pygame.font.SysFont("Arial", 20)

def draw_grid():
    """Draw the grid for the cells."""
    for row in range(ROWS):
        for col in range(COLS):
            pygame.draw.rect(WIN, WHITE, (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE), 1)

def draw_robot():
    """Draw the robot on the screen."""
    WIN.blit(RAHAL_SCALED, (robot_rect.x, robot_rect.y))

def draw_start_goal():
    """Draw the starting position and goal position."""
    pygame.draw.rect(WIN, RED, (robot_rect.x, robot_rect.y, ROBOT_WIDTH, ROBOT_HEIGHT))
    if goal_point:
        pygame.draw.circle(WIN, GREEN, (goal_point[0], goal_point[1]), 10)

def draw_path():
    """Draw the path the robot should follow."""
    for (col, row) in path:
        pygame.draw.rect(WIN, BLUE, (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE))

def draw_mode_selection():
    """Display current mode on the screen."""
    label = font.render(f"Press 'C' for Continuous or 'D' for Discrete", True, BLACK)
    WIN.blit(label, (10, 10))

def draw_dashboard():
    """Display dashboard information."""
    dashboard_text = f"Movement: {movement_type}"
    label = font.render(dashboard_text, True, BLACK)
    WIN.blit(label, (10, HEIGHT - 30))

def draw_potential_field():
    """Draw the potential field based on distances to the robot and goal."""
    if movement_type == "Continuous" and goal_point:
        for row in range(ROWS):
            for col in range(COLS):
                goal_distance = math.hypot(goal_point[0] // CELL_SIZE - col, goal_point[1] // CELL_SIZE - row)
                normalized_potential = min(1, goal_distance / (WIDTH // CELL_SIZE))  # Normalization
                red = int(255 * normalized_potential)  # Higher distance = more red
                color = (red, 0, 0)  # Transition from red to black
                pygame.draw.rect(WIN, color, (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE))

def heuristic(a, b):
    """Calculate heuristic distance."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal):
    """Implement A* pathfinding algorithm."""
    open_set = {start}
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
        
        if current == goal:
            total_path = []
            while current in came_from:
                total_path.append(current)
                current = came_from[current]
            return total_path[::-1]

        open_set.remove(current)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < COLS and 0 <= neighbor[1] < ROWS:
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    open_set.add(neighbor)

    return []

def main():
    global goal_point, movement_type, moving, moving_index, path
    clock = pygame.time.Clock()
    run = True

    while run:
        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    movement_type = "Continuous"  # Switch to Continuous movement
                    path.clear()  # Clear path when switching modes
                elif event.key == pygame.K_d:
                    movement_type = "Discrete"  # Switch to Discrete movement
                    path.clear()  # Clear path when switching modes
                elif event.key == pygame.K_SPACE:
                    moving = True  # Start moving the robot

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    goal_point = event.pos

        WIN.fill(BLACK)  # Clear screen with black
        
        if movement_type == "Discrete":
            draw_grid()  # Draw grid first
            draw_start_goal()  # Draw start and goal next
            draw_path()  # Draw the path

            if moving and goal_point:
                path = a_star((robot_rect.x // CELL_SIZE, robot_rect.y // CELL_SIZE), 
                               (goal_point[0] // CELL_SIZE, goal_point[1] // CELL_SIZE))
                moving_index = 0  # Reset movement index
                moving = False  # Stop moving after setting path

            if moving_index < len(path):
                next_step = path[moving_index]
                next_position = (next_step[0] * CELL_SIZE, next_step[1] * CELL_SIZE)

                robot_rect.topleft = next_position  # Move to the next position
                moving_index += 1  # Move to the next index

        elif movement_type == "Continuous":
            draw_potential_field()  # Draw potential field
            draw_start_goal()  # Draw start and goal in continuous mode

            # Robot movement in continuous mode
            if moving and goal_point:
                goal_col, goal_row = goal_point[0] // CELL_SIZE, goal_point[1] // CELL_SIZE
                robot_col, robot_row = robot_rect.x // CELL_SIZE, robot_rect.y // CELL_SIZE

                # Calculate direction to move towards the goal
                direction_x = goal_col - robot_col
                direction_y = goal_row - robot_row

                distance = math.hypot(direction_x, direction_y)
                if distance > 0:
                    robot_rect.x += (direction_x / distance) * 2  # speed x and y 
                    robot_rect.y += (direction_y / distance) * 2

                # Ensure the robot stays within bounds
                robot_rect.x = max(0, min(robot_rect.x, WIDTH - ROBOT_WIDTH))
                robot_rect.y = max(0, min(robot_rect.y, HEIGHT - ROBOT_HEIGHT))

        draw_robot()  # robot on top 
        draw_mode_selection()  # Draw mode selection text
        draw_dashboard()  # Draw dashboard text

        pygame.display.update()

    pygame.quit()

if __name__ == "__main__":
    main()
