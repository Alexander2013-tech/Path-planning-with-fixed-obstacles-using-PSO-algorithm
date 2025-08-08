import numpy as np

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def to_array(self):
        return np.array([self.x, self.y])

class ObstacleGenerator:
    def __init__(self, r_min=5, r_max=15):
        self.r_min = r_min
        self.r_max = r_max

    def generate_obstacles(self, num, start: np.ndarray, end: np.ndarray):
        # Define dynamic space bounds
        min_bound = np.minimum(start, end) - 10
        max_bound = np.maximum(start, end) + 10
        space_bounds = (min_bound, max_bound)

        centers, radii = [], []
        midpoint = (start + end) / 2
        spread = np.linalg.norm(end - start) / 2

        while len(centers) < num:
            offset = np.random.uniform(-spread / 2, spread / 2, size=2)
            candidate = midpoint + offset
            candidate = np.clip(candidate, space_bounds[0], space_bounds[1])

            radius = np.random.uniform(self.r_min, self.r_max)
            if np.linalg.norm(candidate - start) < radius + 5 or np.linalg.norm(candidate - end) < radius + 5:
                continue
            if all(np.linalg.norm(candidate - c) > (radius + rad) + 5 for c, rad in zip(centers, radii)):
                centers.append(candidate)
                radii.append(radius)

        return centers, radii

class PathValidator:
    @staticmethod
    def is_valid_path(x_spline, y_spline, obstacles, radii):
        for x, y in zip(x_spline, y_spline):
            for center, radius in zip(obstacles, radii):
                if np.linalg.norm([x - center[0], y - center[1]]) < radius:
                    return False
        return True

