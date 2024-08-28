import mesa
from mesa.visualization.modules import CanvasGrid
from mesa.visualization.ModularVisualization import ModularServer
from model import NavigationModel
from agent import RobotAgent, ObstacleAgent

# Function to scale coordinates to the grid size
def scale_coordinate(x, y, grid_width, grid_height):
    # Assuming your coordinate system needs scaling to fit within the grid
    # Adjust these scale factors as needed based on your specific scenario
    x_scaled = (x + abs(min_x)) / (max_x - min_x) * grid_width
    y_scaled = (y + abs(min_y)) / (max_y - min_y) * grid_height
    return x_scaled, y_scaled

# Define the portrayal function to visualize the agents
def agent_portrayal(agent):
    if isinstance(agent, RobotAgent):
        portrayal = {"Shape": "circle", "Color": "blue", "Filled": "true", "Layer": 1, "r": 0.5}
    elif isinstance(agent, ObstacleAgent):
        portrayal = {
            "Shape": "rect",
            "Color": "red",
            "Filled": "true",
            "Layer": 0,
            "w": abs(agent.corners[2] - agent.corners[0]), 
            "h": abs(agent.corners[3] - agent.corners[1])
        }
    else:
        portrayal = {"Shape": "rect", "Color": "white", "Filled": "true", "Layer": 0, "w": 1, "h": 1}
    return portrayal

# Define the grid size
grid = CanvasGrid(agent_portrayal, 30, 20, 600, 400)

# Assuming these are your bounds based on the provided data
min_x, max_x = -3, 3  # Adjust based on your actual coordinate range
min_y, max_y = -3, 3  # Adjust based on your actual coordinate range

# Define the robot positions correctly matching x1 with y1, x2 with y2, etc.
robot_positions = [
    (-2.46190001765768, -0.122421496970765),  # Robot 1 initial position (x1, y1)
    (-0.456652876507227, -2.01699055240998)   # Robot 2 initial position (x2, y2)
]

# Define obstacle corners correctly matching x1 with y1, x2 with y2, etc.
obstacle_corners = [
    [(-1.47524242023976, -0.43581924215234), (-1.47277437230963, -1.04541424601918), (-0.863179368442784, -1.04294619808905), (-0.865647416372916, -0.433351194222208)],  # Obstacle 1
    [(-0.922916487875274, 0.30238608352263), (-1.40386661424334, 0.676951346122546), (-1.77843187684325, 0.196001219754482), (-1.29748175047519, -0.178564042845433)],  # Obstacle 2
    [(0.607198616000506, 0.359034981753366), (0.184783792759392, -0.0804850713797969), (0.624303845892555, -0.502899894620911), (1.04671866913367, -0.0633798414877486)],  # Obstacle 3
    [(0.651444725622291, 1.1960093943409), (0.0419567234128284, 1.18432456816685), (0.0536415495868857, 0.574836565957384), (0.663129551796348, 0.586521392131441)]  # Obstacle 4
]

# Define the target positions correctly matching x1 with y1, x2 with y2, etc.
target_positions = [
    (-1.661884765625, -0.183344483805667),  # Target 1 (x1, y1)
    (-0.882669921875, -0.160709362348177),  # Target 2 (x2, y2)
    (0.0858457031250008, 0.273884969635628),  # Target 3 (x3, y3)
    (0.931095703125001, 0.486655111336033),  # Target 4 (x4, y4)
    (1.045556640625, -0.427803795546557),  # Target 5 (x5, y5)
    (-1.014740234375, 0.713006325910932)   # Target 6 (x6, y6)
]

# Apply scaling to all positions
scaled_robot_positions = [scale_coordinate(x, y, 30, 20) for (x, y) in robot_positions]

scaled_obstacle_corners = [
    [scale_coordinate(x1, y1, 30, 20), scale_coordinate(x2, y2, 30, 20), scale_coordinate(x3, y3, 30, 20), scale_coordinate(x4, y4, 30, 20)]
    for [(x1, y1), (x2, y2), (x3, y3), (x4, y4)] in obstacle_corners
]

scaled_target_positions = [scale_coordinate(x, y, 30, 20) for (x, y) in target_positions]

# Instantiate the model with the provided parameters
model_params = {
    "width": 3.0,  # Grid width
    "height": 2.0,  # Grid height
    "robot_positions": scaled_robot_positions,
    "obstacle_corners": scaled_obstacle_corners,
    "target_positions": scaled_target_positions
}

# Create the server with the model and visualization
server = ModularServer(
    NavigationModel,
    [grid],
    "Warehouse Navigation Model",
    model_params
)

server.port = 8521  # Default port for the server
server.launch()
