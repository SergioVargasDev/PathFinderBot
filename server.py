import matplotlib.pyplot as plt
from model import NavigationModel
from agent import RobotAgent

robot_positions = [
    (-2.46190001765768, -0.122421496970765),
    (-0.456652876507227, -2.01699055240998)
]

obstacle_corners = [
    [(-1.47524242023976, -0.43581924215234), (-1.47277437230963, -1.04541424601918), (-0.863179368442784, -1.04294619808905), (-0.865647416372916, -0.433351194222208)],
    [(-0.922916487875274, 0.30238608352263), (-1.40386661424334, 0.676951346122546), (-1.77843187684325, 0.196001219754482), (-1.29748175047519, -0.178564042845433)],
    [(0.607198616000506, 0.359034981753366), (0.184783792759392, -0.0804850713797969), (0.624303845892555, -0.502899894620911), (1.04671866913367, -0.0633798414877486)],
    [(0.651444725622291, 1.1960093943409), (0.0419567234128284, 1.18432456816685), (0.0536415495868857, 0.574836565957384), (0.663129551796348, 0.586521392131441)]
]

target_positions = [
    (-1.661884765625, -0.183344483805667),
    (-0.882669921875, -0.160709362348177),
    (0.0858457031250008, 0.273884969635628),
    (0.931095703125001, 0.486655111336033),
    (1.045556640625, -0.427803795546557),
    (-1.014740234375, 0.713006325910932)
]

model = NavigationModel(10, 8, robot_positions, obstacle_corners, target_positions)

model.run_simulation(50)

robot_paths = [agent.path_taken for agent in model.schedule.agents if isinstance(agent, RobotAgent)]

plt.figure(figsize=(8, 6))
plt.xlim(-3, 3)
plt.ylim(-2, 2)
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)
plt.gca().spines['bottom'].set_visible(False)
plt.gca().spines['left'].set_visible(False)
plt.xticks(fontsize=12)
plt.yticks(fontsize=12)
plt.xlabel('X [m]', fontsize=14)
plt.ylabel('Y [m]', fontsize=14)
plt.title('Simulation of Robot Movement with A*', fontsize=16)

plt.scatter([p[0] for p in robot_positions], [p[1] for p in robot_positions], color='green', label='Robot Start Positions')

for corners in obstacle_corners:
    x_coords = [c[0] for c in corners]
    y_coords = [c[1] for c in corners]
    plt.fill(x_coords + [x_coords[0]], y_coords + [y_coords[0]], color='red', alpha=0.5)

plt.scatter([p[0] for p in target_positions], [p[1] for p in target_positions], color='blue', marker='x', label='Target Positions')

colors = ['purple', 'orange']
for idx, path in enumerate(robot_paths):
    if path:
        x, y = zip(*path)
        plt.plot(x, y, marker='o', color=colors[idx], label=f'Robot {idx+1} Path')

plt.legend()
plt.show()
