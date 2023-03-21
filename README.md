  <h1 align="center">MWMP-MatLab</h1>
  <h4 align="center">Multi-staged Warm started Motion Planner (MWMP) MatLab library</h4>
  
<p align="center">
  <img src="https://user-images.githubusercontent.com/37618448/177983996-1da1c67d-8037-4b8b-8187-737a8adeee1d.png" width="200">
</p>

![image](https://user-images.githubusercontent.com/37618448/177952812-e9e866cc-04f3-4659-b53b-97cf3950598f.png)


*Author*: [Gonzalo Jesús Paz Delgado](https://github.com/gonzalopd96), gonzalopd96@uma.es

*Supervisor*: [Carlos J. Pérez del Pulgar](https://github.com/carlibiri), carlosperez@uma.es

*Organization*: [Space Robotics Lab, University of Malaga](https://www.uma.es/space-robotics)

## Table of Contents
  * [Description](#description)
  * [Testing and visualization](#testing-and-visualization)
  * [File tree](#file-tree)
  * [Citation](#citation)
  * [Versions](#versions)


## Description

Motion planning library that uses Sequential Linear Quadratic regulator (SLQ) in a Multi-staged Warm-Started manner to plan the movements of a mobile platform. Given the platform kinematic and dynamics model, a description of the scenario, an initial state and a goal, this algorithm plans the motion sequentially:
  - First, Fast Marching Methid (FMM) is used to generate a warm starting trajectory for the mobile base.
  - Second, the unconstrained solution of the motion planning problem is found using Unconstrained SLQ.
  - Third, the unconstrained solution is used to initialize the Constrained SLQ algorithm to find the complete constraints compliant, global motion plan.

Check the [Simulation and field tests video](https://youtu.be/xDFv4Ho4KZs).

  
## Testing and visualization

In the tests/ folder several examples of different systems and models are included. Each test usually includes, first, a "forwardIntegrateSystem.m" function, which basically includes the model, i.e. the discrete differential equations, that define the system, and propagates a given actuation using those equations. Second, each test includes a script (i.e. 19-ExoTeR_torque/ExoTeR_torque-m) that makes use of the library and the system model to generate the motion plan for a platform to reach a goal.

These scripts usually include the following aspects:
- **Initialization**: prepare the workspace for the execution.
- **Initial state and goal**: define the initial and goal states of the platform.
- **Configuration variables**: configure different parameters of the execution (number of timesteps, max iterations, approach to be used, dynamic plotting during the computation...). 
- **Reference trajectory computation**: use of FMM to generate the warm start trajectory.
- **Time horizon estimation**: the time horizon is preestimated in function of the length of the warm start path.
- **Initial reference path adaptation to the state space model**: slightly modifying the trajectory to fit the state space model requirements.
- **Definitive costs**: the costs are modified in function of the time horizon.
- **Generate map info**: fill the structs required by MWMP to use a map of the environment.
- **Generate state space model info**: fill the structs required by MWMP about particular state indexes.
- **Generate trajectory info**: fill the structs required by MWMP about the trajectory.
- **State space model**: generate the x, x0, u and u0 for SLQ with the config.
- **Constraints matrices definition**: generate the constraints matrices C, D, r, G, h.
- **Visualization**: prepare the figures if the dynamic plotting is enabled.
- **SLQR algorithm**: generate the A, B, Q, R matrices and call the SLQ solver iteratively.
- **Plots**: display information about the motion plan into some figures.
- **Simulation**: run the generated motion plan in a simulation with Simscape SimMechanics.
  
## File tree
```bash
MWMP-MatLab/
├── deps/
│   └── ARES-DyMu_matlab/
├── maps/
├── media/
├── simscape/
│   ├── 3D_models/
│   │   └── ExoTeR/
│   ├── ExoTeR.slx
│   ├── ExoTeR_steering.slx
│   ├── ack_3DoF.slx
│   ├── base_3DoF_dynamics_sim.slx
│   ├── base_3DoF_dynamics_sim_forces.slx
│   ├── manipulator3DoF.slx
│   ├── manipulator3DoF_torques.slx
│   └── sim_ExoTeR_torque.slx
├── src/
│   ├── MCM/
│   ├── SLQR/
│   │   ├── SLQ.m
│   │   ├── checkConstraints.m
│   │   └── constrainedSLQ.m
│   ├── costs/
│   ├── maps/
│   ├── models/
│   │   ├── 3DoF/
│   │   └── MA5-E/
│   └── utils/
├── tests/
│   ├── 00-mass_spring_damper/
│   ├── 01-3dof_planar/
│   ├── 10-base_3DoF_dynamics/
│   ├── 10b-base_3DoF_kinematics/
│   ├── 11-platform_inequality/
│   ├── 12-3DoF_dynamics/
│   ├── 13-3DoF_torque/
│   ├── 14-3DoF_inequality/
│   ├── 15-ackermann/
│   ├── 16-5DoF_ExoTeR/
│   ├── 17-ExoTeR_ack/
│   └── 19-ExoTeR_torque/
├── utils/
│   └── performanceStatistics.m
├── .gitignore
├── .gitmodules
├── LICNESE
└── README.md
```

## Citation

If this work was helpful for your research, please consider citing the following BibTeX entry:

@article{author = {G. J. Paz-Delgado and C. J. Pérez-del-Pulgar and M. Azkarate and F. Kirchner and A. García-Cerezo},
   title = {Multi-stage warm started optimal motion planning for over-actuated mobile platforms},
   url = {http://arxiv.org/abs/2207.14659},
   year = {2022}
}


## Versions

[Go to the C++ version](https://github.com/spaceuma/MWMP-Cpp)               
[<img src="https://user-images.githubusercontent.com/37618448/177987095-dc7dba1f-7879-4f9e-a723-b7c4c3780e14.png" width="200">
](https://github.com/spaceuma/MWMP-Cpp)
