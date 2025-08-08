import numpy as np
from scipy.interpolate import splprep, splev
from geometry import PathValidator

class SplinePath:
    def __init__(self, smoothing=2.0):
        self.smoothing = smoothing

    def create_spline(self, start, end, waypoints):
        points = np.vstack([start, waypoints, end]).T
        try:
            tck, _ = splprep(points, s=self.smoothing)
            u = np.linspace(0, 1, 100)
            return splev(u, tck)
        except:
            return None, None

    def path_cost(self, x_spline, y_spline, obstacles, radii):
        if not PathValidator.is_valid_path(x_spline, y_spline, obstacles, radii):
            return 1e6
        dx, dy = np.diff(x_spline), np.diff(y_spline)
        return np.sum(np.sqrt(dx**2 + dy**2))
