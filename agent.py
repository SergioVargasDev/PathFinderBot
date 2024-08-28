from mesa import Agent
import heapq

class RobotAgent(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.path = []
        self.target_positions = []
        self.current_target = None

    def step(self):
        if not self.path:
            if self.current_target is None and self.target_positions:
                self.current_target = self.target_positions.pop(0)
            if self.current_target:
                self.path = self.a_star_search(self.current_target)
        if self.path:
            next_position = self.path.pop(0)
            self.model.grid.move_agent(self, next_position)

    def a_star_search(self, goal):
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        frontier = []
        heapq.heappush(frontier, (0, self.pos))
        came_from = {}
        cost_so_far = {}
        came_from[self.pos] = None
        cost_so_far[self.pos] = 0

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal:
                break

            for next_pos in self.model.grid.get_neighborhood(current, moore=False, include_center=False):
                if self.is_out_of_bounds(next_pos) or self.is_position_occupied_by_obstacle(next_pos):
                    continue
                new_cost = cost_so_far[current] + 1
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + heuristic(goal, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        if current != goal:
            return []

        path = []
        while current != self.pos:
            if current is None:
                return []
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        return path

    def is_position_occupied_by_obstacle(self, pos):
        """Check if the position is within or too close to any obstacles."""
        for obstacle in self.model.schedule.agents:
            if isinstance(obstacle, ObstacleAgent):
                if self.is_near_obstacle(pos, obstacle.corners):
                    return True
        return False

    def is_near_obstacle(self, pos, corners):
        x, y = pos
        x1, y1, x2, y2, x3, y3, x4, y4 = corners
        # Expand the obstacle area by a certain margin
        margin = 0.1
        return (x1 - margin <= x <= x3 + margin) and (y1 - margin <= y <= y4 + margin)

    def is_out_of_bounds(self, pos):
        x, y = pos
        return not (0 <= x < self.model.grid.width and 0 <= y < self.model.grid.height)


class ObstacleAgent(Agent):
    def __init__(self, unique_id, model, corners):
        super().__init__(unique_id, model)
        self.corners = corners  # Store the corners of the obstacle
