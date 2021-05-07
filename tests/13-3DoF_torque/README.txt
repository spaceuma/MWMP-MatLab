Warning: This script "torque_dynamics_3DoF.m" fails to converge. 

The input u(1:3,:) is the arm torque, and the whole state x(1:15,:) is computed in function of that torque. 
The dynamics of the system are thus sequentially computed, from accelerations, speeds to position. This is
a problem, since the actuating torque of iteration k affects the position of the arm in position k+3. Probably,
this doesn't allow the algorithm to converge properly...
