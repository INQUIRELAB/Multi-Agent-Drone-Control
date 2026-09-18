# 🚁 Multi-Agent Drone Control System 🚁

## 📋 Overview

This repository contains MATLAB implementations for multi-agent drone swarm control with obstacle avoidance capabilities and dynamic formation control. The codebase implements various control strategies for drone swarms operating in environments with static and moving obstacles.

## 🌟 Features

- 🔄 Dynamic formation control using Voronoi tessellation
- 🧭 3D path planning with obstacle avoidance
- 🚧 Collision avoidance between agents
- 🏙️ Building/wall constraint handling
- 📊 Visualization tools for drone trajectories and Voronoi diagrams
- 🛡️ Multiple obstacle detection and avoidance strategies

## 🗂️ Repository Structure

The repository is organized into three case studies, each exploring different aspects of multi-agent control:

### CASE STUDY 1
- Basic drone formation control without obstacle avoidance
- Voronoi-based centroid calculation for optimal coverage
- Building/wall constraint handling

### CASE STUDY 2
- Formation control with static and moving obstacles
- Enhanced collision avoidance between agents
- Sensor field-of-view (FOV) based obstacle detection

### CASE STUDY 3
- Full 3D control for drone swarms
- Advanced obstacle avoidance using potential fields
- Spherical obstacle representation and avoidance

## 🛠️ Key Functions

- `Centroid_Calculator.m` - Computes optimal agent positions using Voronoi tessellation
- `dynamics_*.m` - Main simulation files for different scenarios
- `TransMat.m` - Coordinate transformation between reference frames
- `Dyn.m` - Dynamic model of drone agents
- `distance.m` - Distance calculation between agents/obstacles

## 🔬 Mathematical Background

The implementation is based on several key algorithms:
- Lloyd's algorithm for Voronoi-based coverage control
- Artificial potential fields for obstacle avoidance
- Consensus-based flocking control
- Proportional-derivative (PD) controllers for formation maintenance

## 🚀 Getting Started

### Prerequisites
- MATLAB R2020b or newer
- MATLAB Robotics Toolbox (recommended)

### Running a Simulation

1. Clone this repository
2. Open MATLAB and navigate to the repository folder
3. Choose a case study folder and run the corresponding `dynamics_*.m` file
4. Visualize the results using the generated plots

Example:
```matlab
cd 'CASE STUDY 2'
dynamics_multiobs
```

## 📈 Visualization

The code includes visualization tools to display:
- 3D trajectories of all agents
- Voronoi diagrams of agent distribution
- Obstacle positions and avoidance maneuvers
- Relative distances between agents (for collision avoidance verification)

This project is licensed under the PolyForm Noncommercial License 1.0.0 - see the LICENSE file for details.

## License scope

Original INQUIRE Lab code is licensed under PolyForm Noncommercial License 1.0.0. Original INQUIRE Lab datasets, figures, and documentation are licensed under CC BY-NC 4.0. See [LICENSE](LICENSE) for scope and the full license texts. Materials from other rights holders retain their original terms.
