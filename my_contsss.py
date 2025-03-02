from controller import Robot, Motor, Lidar
import matplotlib.pyplot as plt
import numpy as np
import math

MAX_SPEED = 5.24
FORWARD_SPEED = 0.5 * MAX_SPEED
TURN_SPEED = 0.3 * MAX_SPEED
LIDAR_RANGE = 3.0  # Max range of Lidar in meters
LIDAR_RESOLUTION = 360  # Lidar resolution in degrees
OBSTACLE_THRESHOLD = 0.2  # Minimum distance to an obstacle to consider it "in the way"
GRID_SIZE = 100  # 100x100 occupancy grid
CELL_SIZE = 0.1  # 10 cm per grid cell
ARENA_SIZE = GRID_SIZE * CELL_SIZE

robot = Robot()
time_step = int(robot.getBasicTimeStep())

left_wheel = robot.getDevice('left wheel')
right_wheel = robot.getDevice('right wheel')
left_wheel.setPosition(float('inf'))
right_wheel.setPosition(float('inf'))

lidar = robot.getDevice('lidar')
lidar.enable(time_step)

occupancy_grid = np.full((GRID_SIZE, GRID_SIZE), -1)  # -1 = Unknown

def world_to_grid(x, y):
    grid_x = int((x + ARENA_SIZE / 2) / CELL_SIZE)
    grid_y = int((y + ARENA_SIZE / 2) / CELL_SIZE)
    return grid_x, grid_y

def update_occupancy_grid(grid, robot_x, robot_y, lidar_values):
    for i, distance in enumerate(lidar_values):
        if distance < LIDAR_RANGE:
            angle = math.radians(i)  # Convert index to radians
            x = robot_x + distance * math.cos(angle)
            y = robot_y + distance * math.sin(angle)
            grid_x, grid_y = world_to_grid(x, y)
            if 0 <= grid_x < GRID_SIZE and 0 <= grid_y < GRID_SIZE:
                grid[grid_x, grid_y] = 1  # Mark as occupied

    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if grid[i, j] == -1:  # If unexplored
                grid[i, j] = 0  # Mark as explored

state = "FORWARD"

plt.ion()
fig, ax = plt.subplots()
im = ax.imshow(occupancy_grid, cmap='gray', origin='lower')
ax.set_title("Occupancy Grid - Mapping", fontsize=14, fontweight='bold')
plt.axis('off')

while robot.step(time_step) != -1:
    lidar_values = lidar.getRangeImage()

    filtered_lidar_values = [v if v > 0.05 else float('inf') for v in lidar_values]
    min_distance = min(filtered_lidar_values)
    
    print(f"Lidar Min Distance: {min_distance:.2f} m")

    if min_distance < OBSTACLE_THRESHOLD:
        state = "TURN"
    else:
        state = "FORWARD"

    if state == "FORWARD":
        speed = [FORWARD_SPEED, FORWARD_SPEED]
    elif state == "TURN":
        speed = [-TURN_SPEED, TURN_SPEED]

    left_wheel.setVelocity(speed[0])
    right_wheel.setVelocity(speed[1])

    robot_x, robot_y = 0, 0  # Replace these with real robot position
    update_occupancy_grid(occupancy_grid, robot_x, robot_y, filtered_lidar_values)
    
    im.set_data(occupancy_grid)
    im.set_clim(-1, 1)  # Set color limits to handle shading
    plt.draw()
    plt.pause(0.01)
