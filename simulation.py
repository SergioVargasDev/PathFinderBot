import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from robot import Robot

# Define robot start positions, target positions, and obstacle corners
robot_positions = [
    (-2.46190001765768, -0.122421496970765),
    (-0.456652876507227, -1.99968255961363)
]

obstacle_corners = [
    [(-1.47524242023976, -0.43581924215234), (-1.47277437230963, -1.04541424601918), (-0.863179368442784, -1.04294619808905), (-0.865647416372916, -0.433351194222208)],
    [(-0.922916487875274, 0.30238608352263), (-1.40386661424334, 0.676951346122546), (-1.77843187684325, 0.196001219754482), (-1.29748175047519, -0.178564042845433)],
    [(0.607198616000506, 0.359034981753366), (0.184783792759392, -0.0804850713797969), (0.624303845892555, -0.502899894620911), (1.04671866913367, -0.0633798414877486)],
    [(0.651444725622291, 1.1960093943409), (0.0419567234128284, 1.18432456816685), (0.0536415495868857, 0.574836565957384), (0.663129551796348, 0.586521392131441)]
]

# Define target positions for Robot 1
target_positions_robot_1 = [
    (-1.661884765625, -0.183344483805667),
    (-0.882669921875, -0.160709362348177),
    (0.0858457031250008, 0.273884969635628),
    (0.931095703125001, 0.486655111336033),
    (1.045556640625, -0.427803795546557),
    (-1.014740234375, 0.713006325910932)
]

# Define reversed target positions for Robot 2 (Same as Robot 1 but in reverse order)
target_positions_robot_2 = target_positions_robot_1[::-1]

# Initialize robots with obstacle corners
robot1 = Robot(1, robot_positions[0], target_positions_robot_1.copy(), obstacle_corners)
robot2 = Robot(2, robot_positions[1], target_positions_robot_2.copy(), obstacle_corners)

robots = [robot1, robot2]

# Run the simulation
for _ in range(100):  # Adjust the number of steps as needed
    for robot in robots:
        robot.move()

# Save the paths to text files
for robot in robots:
    robot.save_path_to_txt()

# Plot the paths, starting points, and obstacles
plt.figure(figsize=(10, 6))  # Increase the width for adding text on the right
plt.xlim(-3, 3)
plt.ylim(-3, 3)
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Simulation of Robot Movement')

# Plot obstacles
for corners in obstacle_corners:
    x_coords = [c[0] for c in corners]
    y_coords = [c[1] for c in corners]
    plt.fill(x_coords + [x_coords[0]], y_coords + [y_coords[0]], color='red', alpha=0.5)

# Plot robot paths and starting points
plt.plot(*zip(*robot1.path_taken), marker='o', label='Robot 1 Path', color='blue')
plt.scatter(robot1.path_taken[0][0], robot1.path_taken[0][1], color='blue', marker='o', s=100, edgecolor='black', label='Robot 1 Start')

plt.plot(*zip(*robot2.path_taken), marker='o', label='Robot 2 Path', color='orange')
plt.scatter(robot2.path_taken[0][0], robot2.path_taken[0][1], color='orange', marker='o', s=100, edgecolor='black', label='Robot 2 Start')

# Mark target positions for Robot 1 (and Robot 2 since they're the same)
target_xs, target_ys = zip(*target_positions_robot_1)
plt.scatter(target_xs, target_ys, color='green', marker='x', s=150, linewidths=3, label='Target Points')

# Plot circles representing the robot's effective footprint
robot_radius = 0.153  # 15.3 cm as described
for pos in robot1.path_taken:
    circle = Circle(pos, robot_radius, color='blue', alpha=0.1)
    plt.gca().add_patch(circle)

for pos in robot2.path_taken:
    circle = Circle(pos, robot_radius, color='orange', alpha=0.1)
    plt.gca().add_patch(circle)

# Adjust the legend position
plt.legend(loc='upper right')

plt.show()