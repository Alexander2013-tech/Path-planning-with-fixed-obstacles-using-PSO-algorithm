import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import threading

from geometry import ObstacleGenerator
from pso import PSOPathPlanner
from visualizer import PathVisualizer

class PSOPathPlannerApp:
    def __init__(self):
        self.space_bounds = (0, 100)
        self.r_min, self.r_max = 5, 15
        self.root = tk.Tk()
        self.root.title("PSO Path Planning")
        self.root.geometry("960x540")

        self.setup_styles()
        self.setup_ui()

        self.obstacle_generator = ObstacleGenerator(self.r_min, self.r_max)
        self.pso_planner = PSOPathPlanner(space_bounds=self.space_bounds)
        self.convergence_data = []

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#f0f2f5")
        style.configure("TLabel", background="#f0f2f5", font=("Segoe UI", 10))
        style.configure("TButton", font=("Segoe UI", 10, "bold"), padding=6)
        style.configure("TEntry", font=("Segoe UI", 10), padding=4)

    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding=10, style="TFrame")
        main_frame.pack(fill=tk.BOTH, expand=True)

        control_panel = ttk.Frame(main_frame, padding=20, relief="raised", borderwidth=1, style="TFrame")
        control_panel.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        self.canvas_frame = ttk.Frame(main_frame, style="TFrame")
        self.canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        ttk.Label(control_panel, text="PSO Path Planner with\nFixed obstacles", font=("Segoe UI", 14, "bold")).grid(row=0, column=0, columnspan=4, pady=(0, 15))

        # helper for label + entry pair on specific row, col
        def labeled_entry(label, default, row, col):
            ttk.Label(control_panel, text=label).grid(row=row*2+1, column=col*2, sticky="w", pady=(0, 5), padx=(0, 5))
            entry = ttk.Entry(control_panel)
            entry.insert(0, str(default))
            entry.grid(row=row*2+2, column=col*2, columnspan=2, sticky="ew", padx=(0, 5), pady=(0, 10))
            return entry

        control_panel.columnconfigure(0, weight=1)
        control_panel.columnconfigure(1, weight=1)
        control_panel.columnconfigure(2, weight=1)
        control_panel.columnconfigure(3, weight=1)

        self.obs_entry = labeled_entry("Number of Obstacles:", 3, 0, 0)
        self.iter_entry = labeled_entry("Max Iterations:", 400, 0, 1)
        self.sx_entry = labeled_entry("Start X:", 10, 1, 0)
        self.sy_entry = labeled_entry("Start Y:", 10, 1, 1)
        self.ex_entry = labeled_entry("End X:", 90, 2, 0)
        self.ey_entry = labeled_entry("End Y:", 90, 2, 1)

        self.loading_label = ttk.Label(control_panel, text="", foreground="blue", font=("Segoe UI", 10, "italic"))
        self.loading_label.grid(row=7, column=0, columnspan=4, pady=(5, 0))

        self.submit_btn = ttk.Button(control_panel, text="Start Planning", command=self.on_submit)
        self.submit_btn.grid(row=8, column=0, columnspan=4, pady=10, sticky="ew")

        self.convergence_btn = ttk.Button(control_panel, text="Show Convergence", state="disabled", command=self.show_convergence)
        self.convergence_btn.grid(row=9, column=0, columnspan=4, pady=(0, 10), sticky="ew")

        self.visualizer = PathVisualizer(self.canvas_frame, self.space_bounds)


    def on_submit(self):
        def run_planner():
            try:
                n_obs = int(self.obs_entry.get().strip())
                max_iter = int(self.iter_entry.get().strip())
                sx, sy = float(self.sx_entry.get().strip()), float(self.sy_entry.get().strip())
                ex, ey = float(self.ex_entry.get().strip()), float(self.ey_entry.get().strip())
                start = np.array([sx, sy])
                end = np.array([ex, ey])

                if not (self.space_bounds[0] <= sx <= self.space_bounds[1] and
                        self.space_bounds[0] <= sy <= self.space_bounds[1] and
                        self.space_bounds[0] <= ex <= self.space_bounds[1] and
                        self.space_bounds[0] <= ey <= self.space_bounds[1]):
                    raise ValueError("Coordinates must be between 0 and 100.")
                if n_obs < 0:
                    raise ValueError("Number of obstacles must be non-negative.")
                if max_iter <= 0:
                    raise ValueError("Max iterations must be a positive integer.")

                self.submit_btn.config(state="disabled")
                self.convergence_btn.config(state="disabled")
                self.loading_label.config(text="⏳ Planning path, please wait...")

                obstacles, radii = self.obstacle_generator.generate_obstacles(n_obs, start, end)

                # Re-create planner with updated max_iter
                self.pso_planner = PSOPathPlanner(max_iter=max_iter, space_bounds=self.space_bounds)

                result, cost, iter_found, conv = self.pso_planner.optimize(start, end, obstacles, radii)

                def update_ui():
                    self.submit_btn.config(state="normal")
                    self.loading_label.config(text="")
                    self.convergence_data.clear()
                    self.convergence_data.extend(conv)
                    if result[0] is not None:
                        self.convergence_btn.config(state="normal")
                        self.visualizer.animate_path(start, end, *result, obstacles, radii, cost, iter_found)
                    else:
                        messagebox.showwarning("No Path", "No valid path found.")

                self.root.after(10, update_ui)

            except ValueError as e:
                self.submit_btn.config(state="normal")
                self.convergence_btn.config(state="disabled")
                self.loading_label.config(text="")
                messagebox.showerror("Invalid Input", f"Please enter valid numbers.\nDetails: {e}")

        threading.Thread(target=run_planner).start()

    def show_convergence(self):
        if not self.convergence_data:
            messagebox.showwarning("No Data", "Run the planner first to see convergence.")
            return

        conv_window = tk.Toplevel(self.root)
        conv_window.title("PSO Convergence Plot")

        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(self.convergence_data, 'b-', linewidth=2)
        ax.set_title("PSO Algorithm Convergence")
        ax.set_xlabel("Iteration")
        ax.set_ylabel("Best Path Cost")
        ax.grid(True)

        canvas = FigureCanvasTkAgg(fig, master=conv_window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        close_btn = ttk.Button(conv_window, text="Close", command=conv_window.destroy)
        close_btn.pack(pady=10)

    def run(self):
        self.root.mainloop()
