import pygame
import os
import heapq  # for a star 
import random

# Constants
WIDTH, HEIGHT = 900, 900
CELL_SIZE = 50
ROWS, COLS = HEIGHT // CELL_SIZE, WIDTH // CELL_SIZE
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Pathfinding Algorithms: A* vs Dijkstra")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)  # Goal color
RED = (255, 0, 0)  # Start color
BLUE = (0, 0, 255)  # Path color
GRAY = (200, 200, 200)  # Grid line color
FPS = 10
ROBOT_WIDTH, ROBOT_HEIGHT = CELL_SIZE, CELL_SIZE

# Load images
RAHALROBOT = pygame.image.load(os.path.join('1.introduction/images', 'robot_image.jpg'))
RAHAL_SCALED = pygame.transform.scale(RAHALROBOT, (ROBOT_WIDTH, ROBOT_HEIGHT))

# To keep track of robot movement
robot_rect = pygame.Rect(1 * CELL_SIZE, 1 * CELL_SIZE, ROBOT_WIDTH, ROBOT_HEIGHT)  # Starting position in grid
MOVE_SPEED = 1  # 2 to 1 (2 sare3a )
MOVING = False
goal_point = None  # To store the goal point
path = []  # To store the calculated path
algorithm = "A*"  # Default algorithm
steps = 0  # To count the number of steps taken
start_time = 0  # To track the time taken
timing_started = False  # Flag to indicate if timing has started
metrics = {"Path Length": 0, "Time to Goal": 0.0, "Algorithm": algorithm}

# Initialize Pygame font
pygame.font.init()
font = pygame.font.SysFont("Arial", 20)

# Predefined list of static obstacles (as grid cells)
obstacles = [(5, 1), (5, 2), (5, 3), (4, 3), (3, 3), (2, 3), (1, 3),
             (1, 4), (3, 4), (3, 5), (7, 1), (7, 2), (8, 1), (8, 2), (8, 3), (10,3), (11,3),(12,4),
             (12,3), (12,5), (12,6), (12,7), (12,8)]

# Moving obstacles
moving_obstacles = [(2, 6), (7, 8), (7,9)]  # Initial positions of moving obstacles
obstacle_speed = 0.1  # Speed of moving obstacles (very slow)

def draw_grid():
    for row in range(ROWS):
        for col in range(COLS):
            # Draw grid lines
            pygame.draw.rect(WIN, GRAY, (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE), 1)  # Gray grid lines
            if (col, row) in obstacles or (col, row) in moving_obstacles:
                pygame.draw.rect(WIN, BLACK, (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE))  # Fill obstacle cells

def draw_robot():
    WIN.blit(RAHAL_SCALED, (robot_rect.x, robot_rect.y))

def draw_path():
    for (col, row) in path:
        pygame.draw.rect(WIN, BLUE, (col * CELL_SIZE + CELL_SIZE // 4, row * CELL_SIZE + CELL_SIZE // 4, CELL_SIZE // 2, CELL_SIZE // 2))

def draw_start_goal():
    # Draw start point
    pygame.draw.rect(WIN, RED, (robot_rect.x, robot_rect.y, ROBOT_WIDTH, ROBOT_HEIGHT))
    # Draw goal point if defined
    if goal_point:
        pygame.draw.circle(WIN, GREEN, (goal_point[0], goal_point[1]), 10)  # Draw goal point

def draw_algorithm_selection():
    label = font.render(f"Press 'A' for A* or 'D' for Dijkstra's", True, BLACK)
    WIN.blit(label, (10, 10))

def draw_performance_metrics():
    time_taken = metrics["Time to Goal"]
    metrics_display = f"Algorithm: {metrics['Algorithm']} | Steps: {steps} | Time: {time_taken:.2f} sec"
    label = font.render(metrics_display, True, BLACK)
    WIN.blit(label, (10, HEIGHT - 30))

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])   #manhaten 

def a_star(start, goal):
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
            return total_path[::-1]  # Return reversed path

        open_set.remove(current)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < COLS and 0 <= neighbor[1] < ROWS and neighbor not in obstacles and neighbor not in moving_obstacles:
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    open_set.add(neighbor)

    return []  # Return an empty path if no path found

def dijkstra(start, goal):
    queue = []
    heapq.heappush(queue, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while queue:
        current_cost, current = heapq.heappop(queue)

        if current == goal:
            total_path = []
            while current in came_from:
                total_path.append(current)
                current = came_from[current]
            return total_path[::-1]  # Return reversed path

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < COLS and 0 <= neighbor[1] < ROWS and neighbor not in obstacles and neighbor not in moving_obstacles:
                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost
                    heapq.heappush(queue, (priority, neighbor))
                    came_from[neighbor] = current

    return []  # Return an empty path if no path found

def update_moving_obstacles():
    global moving_obstacles
    new_obstacles = []
    for (col, row) in moving_obstacles:
        # Randomly decide to change direction
        if random.random() < 0.05:  # from 0.1% 0.05% chance to change direction
            direction = random.choice([(1, 0), (0, -1), (-1, 0), (0, 1)])  # New random direction
            new_col = col + direction[0]
            new_row = row + direction[1]
        else:
            new_col = col
            new_row = row
        
        # Check bounds and add new position
        if 0 <= new_col < COLS and 0 <= new_row < ROWS and (new_col, new_row) not in obstacles:
            new_obstacles.append((new_col, new_row))
        else:
            new_obstacles.append((col, row))  # Stay in place if out of bounds or hitting a static obstacle

    moving_obstacles = new_obstacles  # Update the moving obstacles

def can_move_to(position):
    # Check if the robot can move to a specified position
    if position in obstacles or position in moving_obstacles:
        return False  # Collision with static or moving obstacles
    return True

def main():
    global MOVING, goal_point, path, algorithm, steps, start_time, timing_started
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
                elif event.key == pygame.K_d:
                    algorithm = "Dijkstra"  # Switch to Dijkstra's algorithm
                elif event.key == pygame.K_SPACE:
                    MOVING = True  # Start moving when space is pressed
                    if goal_point:
                        steps = 0
                        if algorithm == "A*":
                            path = a_star((robot_rect.x // CELL_SIZE, robot_rect.y // CELL_SIZE), (goal_point[0] // CELL_SIZE, goal_point[1] // CELL_SIZE))
                        else:
                            path = dijkstra((robot_rect.x // CELL_SIZE, robot_rect.y // CELL_SIZE), (goal_point[0] // CELL_SIZE, goal_point[1] // CELL_SIZE))
                        metrics["Path Length"] = len(path)  # Store path length
                        metrics["Algorithm"] = algorithm  # Update the displayed algorithm
                        start_time = pygame.time.get_ticks()  # Start timer
                        timing_started = True  # Start timing

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    goal_point = event.pos

        WIN.fill(WHITE)
        draw_grid()  # Draw the grid and obstacles
        if path:
            draw_path()  # Draw the path
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
                    robot_rect.x, robot_rect.y = next_position  #  next position
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
