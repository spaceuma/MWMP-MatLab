# MWMP-MatLab
Multi-staged Warm started Motion Planner (MWMP) MatLab library.

[Get to the C++ version](https://github.com/spaceuma/MWMP-Cpp)

Space Robotics Lab, University of Malaga

Author: Gonzalo Jesús Paz Delgado, gonzalopd96@uma.es

Supervisor: Carlos J. Pérez del Pulgar, carlosperez@uma.es

Motion planning library that uses Sequential Linear Quadratic regulator (SLQ) in a Multi-staged Warm-Started manner to plan the movements of a mobile platform. Given the platform kinematic and dynamics model, a description of the scenario, an initial state and a goal, this algorithm plans the motion sequentially:
  - First, Fast Marching Methid (FMM) is used to generate a warm starting trajectory for the mobile base.
  - Second, the unconstrained solution of the motion planning problem is found using Unconstrained SLQ.
  - Third, the unconstrained solution is used to initialize the Constrained SLQ algorithm to find the complete constraints compliant, global motion plan.
