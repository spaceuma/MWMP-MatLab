function [G, dG] = getG5(joints_position)

    theta1 = joints_position(1);
    theta2 = joints_position(2);
    theta3 = joints_position(3);
    theta4 = joints_position(4);
    theta5 = joints_position(5);
    
    G = zeros(5,1);
    dG = zeros(5,5);
    
    G(1,1) = 2.45e-4*sin(theta1)*(333.0*sin(theta2 + theta3 + theta4) + 5630.0*cos(theta2 + theta3) + 1.4e+4*cos(theta2));
    G(2,1) = 0.404*sin(theta1 + theta2) - 0.404*sin(theta1 - 1.0*theta2) + 2.63*cos(theta1)*sin(theta2) + 1.38*sin(theta2 + theta3)*cos(theta1) - 0.0817*cos(theta2 + theta3)*cos(theta1)*cos(theta4) + 0.0817*sin(theta2 + theta3)*cos(theta1)*sin(theta4);
    G(3,1) = -2.45e-4*cos(theta1)*(333.0*cos(theta2 + theta3 + theta4) - 5630.0*sin(theta2 + theta3));
    G(4,1) = -0.136*cos(theta2 + theta3 + theta4)*cos(theta1);
    G(5,1) = 0;
    
    
    dG(1,1) = 2.45e-4*cos(theta1)*(333.0*sin(theta2 + theta3 + theta4) + 5630.0*cos(theta2 + theta3) + 1.4e+4*cos(theta2));
    dG(1,2) = -2.45e-4*sin(theta1)*(5630.0*sin(theta2 + theta3) - 333.0*cos(theta2 + theta3 + theta4) + 1.4e+4*sin(theta2));
    dG(1,3) = 2.45e-4*sin(theta1)*(333.0*cos(theta2 + theta3 + theta4) - 5630.0*sin(theta2 + theta3));
    dG(1,4) = 0.0817*cos(theta2 + theta3 + theta4)*sin(theta1);
    dG(1,5) = 0;
    dG(2,1) = 0.404*cos(theta1 + theta2) - 0.404*cos(theta1 - 1.0*theta2) - 1.38*sin(theta2 + theta3)*sin(theta1) - 2.63*sin(theta1)*sin(theta2) + 0.0817*cos(theta2 + theta3)*cos(theta4)*sin(theta1) - 0.0817*sin(theta2 + theta3)*sin(theta1)*sin(theta4);
    dG(2,2) = 0.404*cos(theta1 - 1.0*theta2) + 0.404*cos(theta1 + theta2) + 2.63*cos(theta1)*cos(theta2) + 1.38*cos(theta2 + theta3)*cos(theta1) + 0.0817*cos(theta2 + theta3)*cos(theta1)*sin(theta4) + 0.0817*sin(theta2 + theta3)*cos(theta1)*cos(theta4);
    dG(2,3) = 2.45e-4*cos(theta1)*(333.0*sin(theta2 + theta3 + theta4) + 5630.0*cos(theta2 + theta3));
    dG(2,4) = 0.0817*sin(theta2 + theta3 + theta4)*cos(theta1);
    dG(2,5) = 0;
    dG(3,1) = 2.45e-4*sin(theta1)*(333.0*cos(theta2 + theta3 + theta4) - 5630.0*sin(theta2 + theta3));
    dG(3,2) = 2.45e-4*cos(theta1)*(333.0*sin(theta2 + theta3 + theta4) + 5630.0*cos(theta2 + theta3));
    dG(3,3) = 2.45e-4*cos(theta1)*(333.0*sin(theta2 + theta3 + theta4) + 5630.0*cos(theta2 + theta3));
    dG(3,4) = 0.0817*sin(theta2 + theta3 + theta4)*cos(theta1);
    dG(3,5) = 0;
    dG(4,1) = 0.136*cos(theta2 + theta3 + theta4)*sin(theta1);
    dG(4,2) = 0.136*sin(theta2 + theta3 + theta4)*cos(theta1);
    dG(4,3) = 0.136*sin(theta2 + theta3 + theta4)*cos(theta1);
    dG(4,4) = 0.136*sin(theta2 + theta3 + theta4)*cos(theta1);
    dG(4,5) = 0;
    dG(5,1) = 0;
    dG(5,2) = 0;
    dG(5,3) = 0;
    dG(5,4) = 0;
    dG(5,5) = 0;
end

