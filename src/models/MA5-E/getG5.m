function G = getG5(joints_position)

    global d1 a2 a3 d5 d6;
    global m1 m2 m3 m4 m5;

    theta1 = joints_position(1);
    theta2 = joints_position(2);
    theta3 = joints_position(3);
    theta4 = joints_position(4);
    theta5 = joints_position(5);
    
    G = zeros(5,1);
    
    G(1,1) = 4.91*sin(theta1)*(a2*m2*cos(theta2) + 2.0*a2*m3*cos(theta2) + 2.0*a2*m4*cos(theta2) + 2.0*a2*m5*cos(theta2) + d5*m4*sin(theta2 + theta3 + theta4) + a3*m3*cos(theta2 + theta3) + 2.0*a3*m4*cos(theta2 + theta3) + a3*m5*cos(theta2 + theta3));
    G(2,1) = 9.81*m4*cos(theta1)*(a3*sin(theta2 + theta3) + a2*sin(theta2) - 0.5*d5*cos(theta2 + theta3 + theta4)) + 9.81*m3*cos(theta1)*(0.5*a3*sin(theta2 + theta3) + a2*sin(theta2)) + 9.81*m5*cos(theta1)*(0.5*a3*sin(theta2 + theta3) + a2*sin(theta2)) - 2.45*a2*m2*sin(theta1 - 1.0*theta2) + 2.45*a2*m2*sin(theta1 + theta2);
    G(3,1) = 4.91*cos(theta1)*(a3*m3*sin(theta2 + theta3) - 1.0*d5*m4*cos(theta2 + theta3 + theta4) + 2.0*a3*m4*sin(theta2 + theta3) + a3*m5*sin(theta2 + theta3));
    G(4,1) = -4.91*d5*cos(theta2 + theta3 + theta4)*cos(theta1)*(m4 + m5);
    G(5,1) = 0;

    G = G/9.81;
end

