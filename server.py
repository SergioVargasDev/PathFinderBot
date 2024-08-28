import mesa
from mesa.visualization.modules import CanvasGrid
from mesa.visualization.ModularVisualization import ModularServer
from model import NavigationModel
from agent import RobotAgent, ObstacleAgent
import matplotlib.pyplot as plt

# Define the portrayal function to visualize the agents
def agent_portrayal(agent):
    if isinstance(agent, RobotAgent):
        portrayal = {"Shape": "circle", "Color": "blue", "Filled": "true", "Layer": 1, "r": 0.5}
    elif isinstance(agent, ObstacleAgent):
        # Calculate the width and height based on the corners of the obstacle
        x_values = [corner[0] for corner in agent.corners]
        y_values = [corner[1] for corner in agent.corners]
        width = max(x_values) - min(x_values)
        height = max(y_values) - min(y_values)
        
        portrayal = {
            "Shape": "rect",
            "Color": "red",
            "Filled": "true",
            "Layer": 0,
            "w": width,
            "h": height
        }
    else:
        portrayal = {"Shape": "rect", "Color": "white", "Filled": "true", "Layer": 0, "w": 1, "h": 1}
    return portrayal

# Define the grid size to fully cover the coordinate space
grid_width = 6  # To cover x range from -3 to 3
grid_height = 4  # To cover y range from -2 to 2
grid = CanvasGrid(agent_portrayal, grid_width, grid_height, 600, 400)  # Adjust the canvas size accordingly

# Robot and obstacle positions do not need scaling, assuming they are already in the correct coordinate space
robot_positions = [
    (-2.46190001765768, -0.122421496970765),  # Robot 1 initial position
    (-0.456652876507227, -2.01699055240998)   # Robot 2 initial position
]

# Obstacle corners
obstacle_corners = [
    [(-1.47524242023976, -0.43581924215234), (-1.47277437230963, -1.04541424601918), (-0.863179368442784, -1.04294619808905), (-0.865647416372916, -0.433351194222208)],  # Obstacle 1
    [(-0.922916487875274, 0.30238608352263), (-1.40386661424334, 0.676951346122546), (-1.77843187684325, 0.196001219754482), (-1.29748175047519, -0.178564042845433)],  # Obstacle 2
    [(0.607198616000506, 0.359034981753366), (0.184783792759392, -0.0804850713797969), (0.624303845892555, -0.502899894620911), (1.04671866913367, -0.0633798414877486)],  # Obstacle 3
    [(0.651444725622291, 1.1960093943409), (0.0419567234128284, 1.18432456816685), (0.0536415495868857, 0.574836565957384), (0.663129551796348, 0.586521392131441)]  # Obstacle 4
]

# Target positions
target_positions = [
    (-1.661884765625, -0.183344483805667),  # Target 1
    (-0.882669921875, -0.160709362348177),  # Target 2
    (0.0858457031250008, 0.273884969635628),  # Target 3
    (0.931095703125001, 0.486655111336033),  # Target 4
    (1.045556640625, -0.427803795546557),  # Target 5
    (-1.014740234375, 0.713006325910932)   # Target 6
]

# Instantiate the model with the provided parameters
model_params = {
    "width": grid_width,
    "height": grid_height,
    "robot_positions": robot_positions,
    "obstacle_corners": obstacle_corners,
    "target_positions": target_positions
}

# Create the server with the model and visualization
server = ModularServer(
    NavigationModel,
    [grid],
    "Warehouse Navigation Model",
    model_params
)

server.port = 8521  # Default port for the server

# Set up the plot without grid lines or axis lines, but with axis labels
plt.figure(figsize=(8, 6))
plt.xlim(-3, 3)
plt.ylim(-2, 2)
plt.gca().spines['top'].set_visible(False)  # Hide the top spine
plt.gca().spines['right'].set_visible(False)  # Hide the right spine
plt.gca().spines['bottom'].set_visible(False)  # Hide the bottom spine (x-axis)
plt.gca().spines['left'].set_visible(False)  # Hide the left spine (y-axis)
plt.xticks(fontsize=12)
plt.yticks(fontsize=12)
plt.xlabel('X [m]', fontsize=14)
plt.ylabel('Y [m]', fontsize=14)
plt.title('Simulación del movimiento de dos robots con A*', fontsize=16)

# Plot the initial positions of the robots
plt.scatter([p[0] for p in robot_positions], [p[1] for p in robot_positions], color='green', label='Robot Start Positions')

# Plot the obstacles as red rectangles
for corners in obstacle_corners:
    x_coords = [c[0] for c in corners]
    y_coords = [c[1] for c in corners]
    plt.fill(x_coords + [x_coords[0]], y_coords + [y_coords[0]], color='red', alpha=0.5)

# Plot the target positions
plt.scatter([p[0] for p in target_positions], [p[1] for p in target_positions], color='blue', marker='x', label='Target Positions')

plt.legend()
plt.show()

# Run the server
server.launch()
