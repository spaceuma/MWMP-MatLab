  <h1 align="center">MWMP-MatLab</h1>
  <h4 align="center">Multi-staged Warm started Motion Planner (MWMP) MatLab library</h4>
  
<p align="center">
  <img src="https://user-images.githubusercontent.com/37618448/177983996-1da1c67d-8037-4b8b-8187-737a8adeee1d.png" width="200">
</p>

![image](https://user-images.githubusercontent.com/37618448/177952812-e9e866cc-04f3-4659-b53b-97cf3950598f.png)


*Author*: [Gonzalo Jesús Paz Delgado](https://github.com/gonzalopd96), gonzalopd96@uma.es

*Supervisor*: [Carlos J. Pérez del Pulgar](https://github.com/carlibiri), carlosperez@uma.es

[Space Robotics Lab, University of Malaga](https://www.uma.es/robotics-and-mechatronics/info/107542/robotica-espacial/)

## Description

Motion planning library that uses Sequential Linear Quadratic regulator (SLQ) in a Multi-staged Warm-Started manner to plan the movements of a mobile platform. Given the platform kinematic and dynamics model, a description of the scenario, an initial state and a goal, this algorithm plans the motion sequentially:
  - First, Fast Marching Methid (FMM) is used to generate a warm starting trajectory for the mobile base.
  - Second, the unconstrained solution of the motion planning problem is found using Unconstrained SLQ.
  - Third, the unconstrained solution is used to initialize the Constrained SLQ algorithm to find the complete constraints compliant, global motion plan.
  
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

## Versions

[Go to the C++ version](https://github.com/spaceuma/MWMP-Cpp)               
[<img src="https://user-images.githubusercontent.com/37618448/177987095-dc7dba1f-7879-4f9e-a723-b7c4c3780e14.png" width="200">
](https://github.com/spaceuma/MWMP-Cpp)
