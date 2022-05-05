function C = getC3(theta1, theta2, theta3, dtheta1, dtheta2, dtheta3)
    C = zeros(3,3);
    
    C(1,1) = 0;
    C(1,2) = -0.083333*dtheta1*(55.169*sin(2.0*theta2 + theta3) + 77.702*sin(2.0*theta2) + 16.889*sin(theta2 + theta3) + 53.582*sin(theta2) + 8.7169*sin(2.0*theta2 + 2.0*theta3));
    C(1,3) = -0.75*dtheta1*(3.0649*sin(2.0*theta2 + theta3) + 1.8765*sin(theta2 + theta3) + 3.0649*sin(theta3) + 0.96855*sin(2.0*theta2 + 2.0*theta3));
    C(2,1) = 0.041667*dtheta1*(55.169*sin(2.0*theta2 + theta3) + 77.702*sin(2.0*theta2) + 16.889*sin(theta2 + theta3) + 53.582*sin(theta2) + 8.7169*sin(2.0*theta2 + 2.0*theta3));
    C(2,2) = 0;
    C(2,3) = -2.2987*sin(theta3)*(2.0*dtheta2 + dtheta3);
    C(3,1) = 0.375*dtheta1*(3.0649*sin(2.0*theta2 + theta3) + 1.8765*sin(theta2 + theta3) + 3.0649*sin(theta3) + 0.96855*sin(2.0*theta2 + 2.0*theta3));
    C(3,2) = 1.1494*sin(theta3)*(2.0*dtheta2 + dtheta3);
    C(3,3) = -1.1494*dtheta2*sin(theta3);
end

