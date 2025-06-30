# 🚀 Dynamic Window Approach (DWA) Path Planning Simulation

A fully visualized implementation of the **Dynamic Window Approach (DWA)** in Python using Pygame. Simulate and visualize obstacle avoidance and goal-reaching for mobile robots in 2D space — perfect for robotics beginners, path planning researchers, and AI robotics students!

---

## 🧠 What is DWA?

Dynamic Window Approach is a real-time collision avoidance algorithm for mobile robots. It evaluates a set of feasible trajectories based on the robot's dynamic constraints, obstacle proximity, and goal direction.

This project:

- Predicts multiple future trajectories
- Scores each based on distance to goal, speed, and safety
- Selects the best trajectory and visualizes it
- Runs smoothly in a Pygame window with live user interaction!

---

## 🖼️ Features

- 📍 Real-time path planning to a user-defined goal
- ⚠️ Dynamic obstacle addition with right-click
- 🧭 Visualized prediction trajectories
- 🚗 Simulated robot with circular collision model
- 🧪 Easy to tweak robot config for experiments

---

## 🕹️ Demo

### Controls:

- **Left-click** to set a new goal
- **Right-click** to add a new obstacle
- Window will auto-close once robot reaches goal

![https://github.com/yourusername/dwa-python/assets/demo.gif](https://github.com/EricChen0104/DWA_Algorithm_PYTHON/blob/master/assets/MOV%20to%20GIF%20Demo.gif)

---

## 🛠️ Requirements

- Python 3.7+
- Pygame
- NumPy

```bash
pip install pygame numpy
```

## 📦 Run the Simulation

```bash

python DWA.py

```

## Customize Parameters

Easily adjust robot dynamics and cost weights in the Config class:

```bash

self.max_speed = 1.0
self.max_accel = 0.2
self.to_goal_cost_gain = 0.1
self.obstacle_cost_gain = 0.5
...

```

## 🤖 Why Use This?

- ✅ Learn path planning through visualization
- ✅ Great teaching material for robotics classes
- ✅ Easy to hack and extend (e.g., LIDAR input, map loading)
- ✅ No ROS or external frameworks required

## 🌟 Star this repo

If you found this useful or interesting, give it a ⭐ to support more open robotics projects!
