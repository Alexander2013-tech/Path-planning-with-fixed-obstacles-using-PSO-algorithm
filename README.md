# PSO Path Planner with fixed obstacles

This project implements a Particle Swarm Optimization (PSO) algorithm to find an optimized path between two points while avoiding obstacles. The path is modeled using spline interpolation to create smooth curves passing through waypoints.

## Features

- User-configurable number of waypoints.
- User-configurable number of PSO iterations.
- Obstacle avoidance with circular obstacles.
- Interactive GUI using Tkinter and Matplotlib.
- Real-time visualization of the optimized path.
- Multithreaded execution to keep the GUI responsive during optimization.

## Requirements

- Python 3.7+
- numpy
- matplotlib
- scipy
- tkinter (usually included with Python)

Install dependencies:

```bash
pip install numpy matplotlib scipy
```

## Usage

Run the application:

```bash
python main.py
```

- Enter number of waypoints.
- Enter number of iterations.
- Click **Start** to run optimization.
- View optimized path in the plot.
- Check the convergence plot.

## Code Overview

- `PSOPathPlannerApp` — GUI application.
- `pso` — PSO optimization logic.
- `spline_path` — Generates spline path through points.
- `path_cost` — Calculates cost considering distance and obstacle penalties.
-`geometry` — generate the obstacles according to the user input on the gui app.
-`main` — this is the file that has to be run to check the output.


---

Feel free to contribute or open issues!
