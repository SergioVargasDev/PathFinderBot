from mesa import Model
from mesa.time import RandomActivation
from mesa.space import ContinuousSpace
from agent import RobotAgent, ObstacleAgent

class NavigationModel(Model):
    def __init__(self, width, height, robot_positions, obstacle_corners, target_positions):
        super().__init__()
        self.grid = ContinuousSpace(width, height, False)
        self.schedule = RandomActivation(self)

        # Add obstacles
        for corners in obstacle_corners:
            obstacle = ObstacleAgent(self.next_id(), self, corners)
            center_position = (
                (corners[0][0] + corners[2][0]) / 2,
                (corners[0][1] + corners[2][1]) / 2
            )
            self.grid.place_agent(obstacle, center_position)
            self.schedule.add(obstacle)

        # Add robots
        for i, pos in enumerate(robot_positions):
            robot = RobotAgent(self.next_id(), self)
            robot.target_positions = target_positions.copy()  # Assign target positions
            self.grid.place_agent(robot, pos)
            self.schedule.add(robot)

    def step(self):
        self.schedule.step()
        # Add logic to export the robot paths to a txt file if needed.
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
