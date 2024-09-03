import heapq

class Robot:
    def __init__(self, id, start_pos, targets, obstacle_corners, robot_size=(0.16, 0.20), step_size=0.1):
        self.id = id
        self.position = start_pos
        self.targets = targets
        self.step_size = step_size
        self.path_taken = [start_pos]
        self.current_target = None
        self.robot_radius = 0.153  # Robot's maximum radius based on given dimensions
        self.obstacle_corners = [self.expand_obstacle(corners, self.robot_radius) for corners in obstacle_corners]  # Expand obstacles

    def expand_obstacle(self, corners, margin):
        # Expand each obstacle by the robot's radius (adding a margin of safety)
        expanded_corners = []
        for x, y in corners:
            expanded_corners.append((x - margin, y - margin))
            expanded_corners.append((x + margin, y + margin))
        return expanded_corners

    def move(self):
        if not self.current_target and self.targets:
            self.current_target = self.targets.pop(0)

        if self.current_target:
            path = self.a_star_search(self.current_target)
            if path and len(path) > 1:  # Ensure that there's a next position to move to
                next_position = path[1]
                self.position = next_position
                self.path_taken.append(next_position)
                print(f"Robot {self.id} moved to position {next_position}")
            else:
                print(f"Robot {self.id} reached its target at position {self.position}")
                self.current_target = None  # Move to the next target
        else:
            print(f"Robot {self.id} has no more targets.")

    def a_star_search(self, goal):
        def heuristic(a, b):
            return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

        frontier = []
        heapq.heappush(frontier, (0, self.position))
        came_from = {}
        cost_so_far = {}
        came_from[self.position] = None
        cost_so_far[self.position] = 0

        while frontier:
            current = heapq.heappop(frontier)[1]

            if self.distance(current, goal) <= self.step_size:
                return self.reconstruct_path(came_from, current)

            for next_pos in self.get_neighbors(current):
                if self.is_position_occupied_by_obstacle(next_pos):
                    continue

                new_cost = cost_so_far[current] + self.distance(current, next_pos)
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + heuristic(goal, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current

        return []

    def reconstruct_path(self, came_from, current):
        path = []
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        return path

    def distance(self, a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def get_neighbors(self, pos):
        neighbors = []
        for dx in [-self.step_size, 0, self.step_size]:
            for dy in [-self.step_size, 0, self.step_size]:
                if dx != 0 or dy != 0:
                    neighbor = (pos[0] + dx, pos[1] + dy)
                    neighbors.append(neighbor)
        return neighbors

    def is_position_occupied_by_obstacle(self, pos):
        for corners in self.obstacle_corners:
            if self.is_inside_polygon(pos, corners):
                return True
        return False

    def is_inside_polygon(self, pos, corners):
        x, y = pos
        n = len(corners)
        inside = False

        p1x, p1y = corners[0]
        for i in range(n + 1):
            p2x, p2y = corners[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    def save_path_to_txt(self):
        with open(f'robot_{self.id}_path.txt', 'w') as f:
            for position in self.path_taken:
                f.write(f"{position}\n")
        print(f"Robot {self.id} path saved to robot_{self.id}_path.txt")