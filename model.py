from mesa import Model
from mesa.time import RandomActivation
from mesa.space import MultiGrid  # Use MultiGrid for a grid-based environment
from agent import RobotAgent, ObstacleAgent

class NavigationModel(Model):
    def __init__(self, width, height, robot_positions, obstacle_corners, target_positions):
        super().__init__()
        self.grid = MultiGrid(width, height, False)
        self.schedule = RandomActivation(self)

        # Add obstacles
        for corners in obstacle_corners:
            obstacle = ObstacleAgent(self.next_id(), self, corners)
            center_position = self.calculate_center_position(corners)
            self.grid.place_agent(obstacle, self.grid_to_cell(center_position))
            self.schedule.add(obstacle)

        # Add robots
        for i, pos in enumerate(robot_positions):
            robot = RobotAgent(self.next_id(), self)
            robot.target_positions = target_positions.copy()  # Assign target positions
            self.grid.place_agent(robot, self.grid_to_cell(pos))
            self.schedule.add(robot)

    def calculate_center_position(self, corners):
        """Calculate the center of the obstacle."""
        x_coords = [corners[i][0] for i in range(4)]
        y_coords = [corners[i][1] for i in range(4)]
        center_x = sum(x_coords) / 4
        center_y = sum(y_coords) / 4
        return (center_x, center_y)

    def grid_to_cell(self, pos):
        """Convert the position into a cell that fits within the grid bounds."""
        x, y = pos
        cell_x = min(max(int(round(x)), 0), self.grid.width - 1)
        cell_y = min(max(int(round(y)), 0), self.grid.height - 1)
        return cell_x, cell_y

    def step(self):
        self.schedule.step()
        self.export_robot_paths()

    def export_robot_paths(self):
        data = []
        for agent in self.schedule.agents:
            if isinstance(agent, RobotAgent):
                data.append({
                    'initial_position': agent.pos,
                    'path_taken': agent.path
                })
        with open('robot_paths.txt', 'w') as f:
            for d in data:
                f.write(f"{d['initial_position']}: {d['path_taken']}\n")
