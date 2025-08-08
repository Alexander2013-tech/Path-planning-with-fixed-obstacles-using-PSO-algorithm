import numpy as np
from spline_path import SplinePath

class PSOPathPlanner:
    def __init__(self, n_waypoints=4, n_particles=200, max_iter=400, w=0.5, c1=1.5, c2=1.5, space_bounds=(0, 100)):
        self.n_waypoints = n_waypoints
        self.n_particles = n_particles
        self.max_iter = max_iter
        self.w = w
        self.c1 = c1
        self.c2 = c2
        self.space_bounds = space_bounds
        self.spline_path = SplinePath()

    def optimize(self, start, end, obstacles, radii):
        dim = self.n_waypoints * 2
        bounds = self.space_bounds
        positions = np.random.uniform(bounds[0], bounds[1], (self.n_particles, dim))
        velocities = np.random.uniform(-1, 1, (self.n_particles, dim))
        p_best_pos = positions.copy()
        p_best_cost = np.full(self.n_particles, np.inf)
        g_best_pos = None
        g_best_cost = np.inf
        g_best_iter = -1
        convergence = []

        for iteration in range(self.max_iter):
            for i in range(self.n_particles):
                waypoints = positions[i].reshape(self.n_waypoints, 2)
                x_spline, y_spline = self.spline_path.create_spline(start, end, waypoints)
                if x_spline is None:
                    continue
                cost = self.spline_path.path_cost(x_spline, y_spline, obstacles, radii)
                if cost < p_best_cost[i]:
                    p_best_cost[i] = cost
                    p_best_pos[i] = positions[i].copy()
                if cost < g_best_cost:
                    g_best_cost = cost
                    g_best_pos = positions[i].copy()
                    g_best_iter = iteration

            convergence.append(g_best_cost)
            print(f"Iteration {iteration + 1}/{self.max_iter} - Best Cost: {g_best_cost:.4f}")

            for i in range(self.n_particles):
                r1, r2 = np.random.rand(2)
                velocities[i] = (self.w * velocities[i] +
                                 self.c1 * r1 * (p_best_pos[i] - positions[i]) +
                                 self.c2 * r2 * (g_best_pos - positions[i]))
                positions[i] += velocities[i]
                positions[i] = np.clip(positions[i], bounds[0], bounds[1])

        if g_best_pos is not None:
            best_waypoints = g_best_pos.reshape(self.n_waypoints, 2)
            x_spline, y_spline = self.spline_path.create_spline(start, end, best_waypoints)
            return (x_spline, y_spline), g_best_cost, g_best_iter, convergence
        return None, np.inf, -1, convergence
