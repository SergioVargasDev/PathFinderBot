from mesa import Agent
import heapq
import math

class RobotAgent(Agent):
    def __init__(self, unique_id, model, target_positions, step_size=0.01):
        super().__init__(unique_id, model)
        self.path = []
        self.target_positions = target_positions
        self.current_target = None
        self.path_taken = []
        self.step_size = step_size

    def step(self):
        if not self.path:
            if self.current_target is None and self.target_positions:
                self.current_target = self.target_positions.pop(0)
            if self.current_target:
                self.path = self.a_star_search(self.current_target)
        if self.path:
            next_position = self.path.pop(0)
            self.model.grid.move_agent(self, next_position)
            self.path_taken.append(next_position)
            print(f"Robot {self.unique_id} moved to position {next_position}")
        else:
            print(f"Robot {self.unique_id} is waiting at position {self.pos}")

    def a_star_search(self, goal):
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        frontier = []
        heapq.heappush(frontier, (0, self.pos))
        came_from = {}
        cost_so_far = {}
        came_from[self.pos] = None
        cost_so_far[self.pos] = 0

        while frontier:
            current = heapq.heappop(frontier)[1]

            if heuristic(current, goal) < self.step_size:
                break

            for next_pos in self.get_neighbors(current):
                if self.is_out_of_bounds(next_pos) or self.is_position_occupied_by_obstacle(next_pos) or self.is_position_occupied_by_robot(next_pos):
                    continue

                new_cost = cost_so_far[current] + heuristic(current, next_pos)
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + heuristic(goal, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        if heuristic(current, goal) >= self.step_size:
            print(f"Robot {self.unique_id} failed to find a path from {self.pos} to {goal}.")
            return []

        path = []
        while current != self.pos:
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        print(f"Robot {self.unique_id} found a path: {path}")
        return path

    def get_neighbors(self, pos):
        x, y = pos
        neighbors = []
        steps = [-self.step_size, 0, self.step_size]
        for dx in steps:
            for dy in steps:
                if dx != 0 or dy != 0:
                    neighbor = (round(x + dx, 8), round(y + dy, 8))
                    neighbors.append(neighbor)
        return neighbors

    def is_position_occupied_by_obstacle(self, pos):
        for obstacle in self.model.schedule.agents:
            if isinstance(obstacle, ObstacleAgent):
                if self.is_near_obstacle(pos, obstacle.corners):
                    return True
        return False
    
    def is_position_occupied_by_robot(self, pos):
        for agent in self.model.schedule.agents:
            if isinstance(agent, RobotAgent) and agent.pos == pos:
                return True
        return False

    def is_near_obstacle(self, pos, corners):
        x, y = pos
        x_coords = [corner[0] for corner in corners]
        y_coords = [corner[1] for corner in corners]

        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)

        margin = 0.01

        return (min_x - margin <= x <= max_x + margin) and (min_y - margin <= y <= max_y + margin)

    def is_out_of_bounds(self, pos):
        x, y = pos
        return not (0 <= x <= self.model.grid.width and 0 <= y <= self.model.grid.height)


class ObstacleAgent(Agent):
    def __init__(self, unique_id, model, corners):
        super().__init__(unique_id, model)
        self.corners = corners
