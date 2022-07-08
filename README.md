# MWMP-MatLab
Multi-staged Warm started Motion Planner (MWMP) MatLab library

![image](https://user-images.githubusercontent.com/37618448/177952812-e9e866cc-04f3-4659-b53b-97cf3950598f.png)


*Author*: [Gonzalo Jesús Paz Delgado](https://github.com/gonzalopd96), gonzalopd96@uma.es

*Supervisor*: [Carlos J. Pérez del Pulgar](https://github.com/carlibiri), carlosperez@uma.es

[Space Robotics Lab, University of Malaga](https://www.uma.es/robotics-and-mechatronics/info/107542/robotica-espacial/)

## Description

Motion planning library that uses Sequential Linear Quadratic regulator (SLQ) in a Multi-staged Warm-Started manner to plan the movements of a mobile platform. Given the platform kinematic and dynamics model, a description of the scenario, an initial state and a goal, this algorithm plans the motion sequentially:
  - First, Fast Marching Methid (FMM) is used to generate a warm starting trajectory for the mobile base.
  - Second, the unconstrained solution of the motion planning problem is found using Unconstrained SLQ.
  - Third, the unconstrained solution is used to initialize the Constrained SLQ algorithm to find the complete constraints compliant, global motion plan.

## Versions

[Go to the C++ version](https://github.com/spaceuma/MWMP-Cpp)               
[<img src="https://user-images.githubusercontent.com/37618448/177987095-dc7dba1f-7879-4f9e-a723-b7c4c3780e14.png" width="200">
](https://github.com/spaceuma/MWMP-MatLab)
