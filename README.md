# PDM_5 - Motion Planning for Autonomous Vehicles

## Overview
This project implements motion planning algorithms for autonomous vehicles, including:
- Rapidly Exploring Random Trees (RRT*)
- Lattice Planner (A* search-based)

The system simulates a Prius robot navigating through an environment using a custom-defined motion planning pipeline.

## File Overview:
- RRT.py
- Lattice.py #generates set of potential trajectories then detrmines the path with A* to the goal
- .gitignore
- analysis.py
- car_data.py
- prius_test.py
- lattice_test.py #scenario with lattice


## Features
- RRT* and Lattice motion planners
- Prius vehicle simulation with kinematic constraints
- Visual trajectory analysis

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/mauricemerced/PDM_5.git
   cd PDM_5

