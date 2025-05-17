import pygame
import os
import heapq  # for A* 
import random

# Constants
WIDTH, HEIGHT = 900, 900
CELL_SIZE = 50
ROWS, COLS = HEIGHT // CELL_SIZE, WIDTH // CELL_SIZE
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Pathfinding Algorithm: A* with Cost Visualization")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)  # Goal color
RED = (255, 0, 0)  # Start color
BLUE = (0, 0, 255)  # Path color
GRAY = (200, 200, 200)  # Grid line color
FPS = 30
ROBOT_WIDTH, ROBOT_HEIGHT = CELL_SIZE, CELL_SIZE

# Load images
RAHALROBOT = pygame.image.load(os.path.join('1.introduction/images', 'robot_image.jpg'))
RAHAL_SCALED = pygame.transform.scale(RAHALROBOT, (ROBOT_WIDTH, ROBOT_HEIGHT))

# To keep track of robot movement
robot_rect = pygame.Rect(1 * CELL_SIZE, 1 * CELL_SIZE, ROBOT_WIDTH, ROBOT_HEIGHT)  # Starting position in grid
MOVE_SPEED = 1
MOVING = False
goal_point = None
path = []
algorithm = "A*"  # Only A* algorithm
steps = 0
start_time = 0
timing_started = False
metrics = {"Path Length": 0, "Time to Goal": 0.0, "Algorithm": algorithm}

# Initialize costs grid
cost_grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]

# Predefined list of static obstacles (as grid cells)
obstacles = [(5, 1), (5, 2), (5, 3), (4, 3), (3, 3), (2, 3), (1, 3),
             (1, 4), (3, 4), (3, 5), (7, 1), (7, 2), (8, 1), (8, 2), (8, 3), (10, 3), (11, 3), (12, 4),
             (12, 3), (12, 5), (12, 6), (12, 7), (12, 8)]

# Moving obstacles
moving_obstacles = [(2, 6), (7, 8), (7, 9)]  # Initial positions of moving obstacles
obstacle_speed = 0.1  # Speed of moving obstacles

def draw_grid():
    for row in range(ROWS):
        for col in range(COLS):
            # Draw grid lines
            pygame.draw.rect(WIN, GRAY, (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE), 1)  # Gray grid lines
            # Draw cell based on its cost
            cost_color = cost_to_color(cost_grid[row][col])
            pygame.draw.rect(WIN, cost_color, (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE))

            if (col, row) in obstacles or (col, row) in moving_obstacles:
                pygame.draw.rect(WIN, BLACK, (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE))  # Fill obstacle cells

def cost_to_color(cost):
    # Normalize cost for coloring
    normalized_cost = max(0, min(cost, 5))  # Clamping cost between 0 and 5
    intensity = int((1 - (normalized_cost / 5)) * 255)  # Darker for higher cost
    return (intensity, intensity, intensity)  # Return grayscale color

def draw_robot():
    WIN.blit(RAHAL_SCALED, (robot_rect.x, robot_rect.y))

def draw_path():
    for (col, row) in path:
        pygame.draw.rect(WIN, BLUE, (col * CELL_SIZE + CELL_SIZE // 4, row * CELL_SIZE + CELL_SIZE // 4, CELL_SIZE // 2, CELL_SIZE // 2))

def draw_start_goal():
    pygame.draw.rect(WIN, RED, (robot_rect.x, robot_rect.y, ROBOT_WIDTH, ROBOT_HEIGHT))  # Draw start point
    if goal_point:
        pygame.draw.circle(WIN, GREEN, (goal_point[0], goal_point[1]), 10)  # Draw goal point

def draw_algorithm_selection():
    label = font.render(f"Press 'A' for A*", True, BLACK)
    WIN.blit(label, (10, 10))

def draw_performance_metrics():
    time_taken = metrics["Time to Goal"]
    metrics_display = f"Algorithm: {metrics['Algorithm']} | Steps: {steps} | Time: {time_taken:.2f} sec"
    label = font.render(metrics_display, True, BLACK)
    WIN.blit(label, (10, HEIGHT - 30))

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

def a_star(start, goal):
    open_set = {start}
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal) + cost_grid[start[1]][start[0]]}

    while open_set:
        current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
        
        if current == goal:
            total_path = []
            while current in came_from:
                total_path.append(current)
                current = came_from[current]
            return total_path[::-1]  # Return reversed path

        open_set.remove(current)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < COLS and 0 <= neighbor[1] < ROWS and neighbor not in obstacles and neighbor not in moving_obstacles:
                tentative_g_score = g_score[current] + 1 + cost_grid[neighbor[1]][neighbor[0]]
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    open_set.add(neighbor)

    return []  # Return an empty path if no path found

def update_costs():
    for row in range(ROWS):
        for col in range(COLS):
            if (col, row) not in obstacles and (col, row) not in moving_obstacles:
                cost_grid[row][col] = random.uniform(0, 5)  # Assign a random cost between 0 and 5

def update_moving_obstacles():
    global moving_obstacles
    new_obstacles = []
    for (col, row) in moving_obstacles:
        if random.random() < 0.05:  # Chance to change direction
            direction = random.choice([(1, 0), (0, -1), (-1, 0), (0, 1)])  # New random direction
            new_col = col + direction[0]
            new_row = row + direction[1]
        else:
            new_col = col
            new_row = row
        
        if 0 <= new_col < COLS and 0 <= new_row < ROWS and (new_col, new_row) not in obstacles:
            new_obstacles.append((new_col, new_row))
        else:
            new_obstacles.append((col, row))  # Stay in place if out of bounds or hitting a static obstacle

    moving_obstacles = new_obstacles  # Update the moving obstacles

def can_move_to(position):
    if position in obstacles or position in moving_obstacles:
        return False  # Collision with static or moving obstacles
    return True

def main():
    global MOVING, goal_point, path, steps, start_time
    clock = pygame.time.Clock()
    run = True
    while run:
        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_a:
                    algorithm = "A*"  # Switch to A* algorithm
                elif event.key == pygame.K_SPACE:
                    MOVING = True  # Start moving when space is pressed
                    if goal_point:
                        update_costs()  # Calculate costs when moving starts
                        steps = 0
                        path = a_star((robot_rect.x // CELL_SIZE, robot_rect.y // CELL_SIZE), (goal_point[0] // CELL_SIZE, goal_point[1] //CELL_SIZE))
                        metrics["Path Length"] = len(path)  # Store path length
                        metrics["Algorithm"] = algorithm  # Update the displayed algorithm
                        start_time = pygame.time.get_ticks()  # Start timer

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    goal_point = event.pos

        WIN.fill(WHITE)
        draw_grid()  # Draw the grid and costs
        if path:
            draw_path()  # Draw the path if it exists
        draw_start_goal()  # Draw start and goal points
        draw_robot()  # Draw the robot
        draw_algorithm_selection()  # Show algorithm selection
        draw_performance_metrics()  # Show performance metrics

        update_moving_obstacles()  # Update positions of moving obstacles

        if MOVING and path:
            if path:
                next_step = path[0]  # Get next step from path
                next_position = (next_step[0] * CELL_SIZE, next_step[1] * CELL_SIZE)

                # Check for collision with moving obstacles
                if can_move_to((next_step[0], next_step[1])):
                    robot_rect.x, robot_rect.y = next_position  # Move to next position
                    path.pop(0)  # Remove step from path
                    steps += 1  # Increment step count
                    
                    if not path:  # If the robot reaches the goal
                        end_time = pygame.time.get_ticks()
                        metrics["Time to Goal"] = (end_time - start_time) / 1000.0  # Time in seconds
                        MOVING = False  # Stop moving when the goal is reached

        pygame.display.update()

    pygame.quit()

if __name__ == "__main__":
    main()
