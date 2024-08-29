from mesa import Model
from mesa.time import RandomActivation
from mesa.space import ContinuousSpace
from agent import RobotAgent, ObstacleAgent

class NavigationModel(Model):
    def __init__(self, width, height, robot_positions, obstacle_corners, target_positions, step_size=0.01):
        super().__init__()
        self.space = ContinuousSpace(width, height, False)
        self.schedule = RandomActivation(self)
        self.step_size = step_size

        # Add obstacles
        for idx, corners in enumerate(obstacle_corners):
            obstacle = ObstacleAgent(self.next_id(), self, corners)
            center_position = self.calculate_center_position(corners)
            print(f"Placing obstacle {idx+1} at position {center_position} with corners {corners}")
            self.space.place_agent(obstacle, center_position)
            self.schedule.add(obstacle)

        # Add robots
        robot1 = RobotAgent(1, self, target_positions.copy(), step_size=self.step_size)
        robot2 = RobotAgent(2, self, target_positions[::-1].copy(), step_size=self.step_size)

        print(f"Placing Robot 1 at position {robot_positions[0]} with targets {target_positions}")
        self.space.place_agent(robot1, robot_positions[0])
        self.schedule.add(robot1)

        print(f"Placing Robot 2 at position {robot_positions[1]} with targets {target_positions[::-1]}")
        self.space.place_agent(robot2, robot_positions[1])
        self.schedule.add(robot2)

    def calculate_center_position(self, corners):
        x_coords = [corner[0] for corner in corners]
        y_coords = [corner[1] for corner in corners]
        center_x = sum(x_coords) / len(corners)
        center_y = sum(y_coords) / len(corners)
        return (center_x, center_y)

    def step(self):
        self.schedule.step()

    def run_simulation(self, steps):
        for _ in range(steps):
            self.step()
        self.export_robot_paths()

    def export_robot_paths(self):
        for agent in self.schedule.agents:
            if isinstance(agent, RobotAgent):
                with open(f'robot_{agent.unique_id}_path.txt', 'w') as f:
                    f.write(f"Robot {agent.unique_id} path: {agent.path_taken}\n")
