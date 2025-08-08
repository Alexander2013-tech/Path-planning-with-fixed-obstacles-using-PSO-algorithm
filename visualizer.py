import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import tkinter as tk

class PathVisualizer:
    def __init__(self, canvas_frame, space_bounds=(0, 100)):
        self.canvas_frame = canvas_frame
        self.space_bounds = space_bounds
        self.fig = None
        self.ax = None
        self.canvas = None
        self.robot = None
        self.ani = None

    def clear_canvas(self):
        for widget in self.canvas_frame.winfo_children():
            widget.destroy()

    def animate_path(self, start, end, x_spline, y_spline, obstacles, radii, best_cost, best_iteration):
        self.clear_canvas()

        self.fig, self.ax = plt.subplots(figsize=(8, 8), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.canvas_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.robot, = self.ax.plot([], [], 'b-', lw=2)
        self.ax.plot(start[0], start[1], 'go', label='Start')
        self.ax.plot(end[0], end[1], 'ro', label='End')

        for center, radius in zip(obstacles, radii):
            circle = plt.Circle(center, radius, color='gray', alpha=0.6)
            self.ax.add_patch(circle)

        self.ax.set_xlim(self.space_bounds[0], self.space_bounds[1])
        self.ax.set_ylim(self.space_bounds[0], self.space_bounds[1])
        self.ax.set_title(f"Path Cost: {best_cost:.2f} @ Iter {best_iteration}")
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=3)
        self.fig.subplots_adjust(bottom=0.2)

        def init():
            self.robot.set_data([], [])
            return self.robot,

        def update(i):
            self.robot.set_data(x_spline[:i + 1], y_spline[:i + 1])
            return self.robot,

        self.ani = animation.FuncAnimation(
            self.fig, update, frames=len(x_spline),
            init_func=init, blit=True, interval=60, repeat=False
        )

        self.canvas.draw()
