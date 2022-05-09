function C = getC5(joints_position, joints_speed)

    global d1 a2 a3 d5 d6;
    global m1 m2 m3 m4 m5;
    global width1 width2 width3 width4 width5;

    theta1 = joints_position(1);
    theta2 = joints_position(2);
    theta3 = joints_position(3);
    theta4 = joints_position(4);
    theta5 = joints_position(5);
    
    dtheta1 = joints_speed(1);
    dtheta2 = joints_speed(2);
    dtheta3 = joints_speed(3);
    dtheta4 = joints_speed(4);
    dtheta5 = joints_speed(5);

    C = zeros(5,5);
    
    C(1,1) = 0;
    C(1,2) = 0.333*d5^2*dtheta1*m4*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.0833*d6^2*dtheta1*m5*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.25*dtheta1*m4*width4^2*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.25*dtheta1*m5*width5^2*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.5*dtheta5*m5*width5^2*sin(theta2 + theta3 + theta4) - 0.25*a2^2*dtheta1*m2*sin(2.0*theta2) - 1.0*a2^2*dtheta1*m3*sin(2.0*theta2) - 1.0*a2^2*dtheta1*m4*sin(2.0*theta2) - 1.0*a2^2*dtheta1*m5*sin(2.0*theta2) - 0.25*a3^2*dtheta1*m3*sin(2.0*theta2 + 2.0*theta3) - 1.0*a3^2*dtheta1*m4*sin(2.0*theta2 + 2.0*theta3) - 0.25*a3^2*dtheta1*m5*sin(2.0*theta2 + 2.0*theta3) - 1.0*a2*a3*dtheta1*m3*sin(2.0*theta2 + theta3) - 2.0*a2*a3*dtheta1*m4*sin(2.0*theta2 + theta3) - 1.0*a2*a3*dtheta1*m5*sin(2.0*theta2 + theta3) + a3*d5*dtheta1*m4*cos(2.0*theta2 + 2.0*theta3 + theta4) + a2*d5*dtheta1*m4*cos(2.0*theta2 + theta3 + theta4);
    C(1,3) = 0.333*d5^2*dtheta1*m4*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.0833*d6^2*dtheta1*m5*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.25*dtheta1*m4*width4^2*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.25*dtheta1*m5*width5^2*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.5*dtheta5*m5*width5^2*sin(theta2 + theta3 + theta4) - 0.25*a3^2*dtheta1*m3*sin(2.0*theta2 + 2.0*theta3) - 1.0*a3^2*dtheta1*m4*sin(2.0*theta2 + 2.0*theta3) - 0.25*a3^2*dtheta1*m5*sin(2.0*theta2 + 2.0*theta3) - 0.5*a2*a3*dtheta1*m3*sin(2.0*theta2 + theta3) - 1.0*a2*a3*dtheta1*m4*sin(2.0*theta2 + theta3) - 0.5*a2*a3*dtheta1*m5*sin(2.0*theta2 + theta3) + a3*d5*dtheta1*m4*cos(2.0*theta2 + 2.0*theta3 + theta4) + 0.5*a2*d5*dtheta1*m4*cos(theta3 + theta4) + 0.5*a2*d5*dtheta1*m4*cos(2.0*theta2 + theta3 + theta4) - 0.5*a2*a3*dtheta1*m3*sin(theta3) - 1.0*a2*a3*dtheta1*m4*sin(theta3) - 0.5*a2*a3*dtheta1*m5*sin(theta3);
    C(1,4) = 0.333*d5^2*dtheta1*m4*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.0833*d6^2*dtheta1*m5*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.25*dtheta1*m4*width4^2*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.25*dtheta1*m5*width5^2*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.5*dtheta5*m5*width5^2*sin(theta2 + theta3 + theta4) + 0.5*a3*d5*dtheta1*m4*cos(2.0*theta2 + 2.0*theta3 + theta4) + 0.5*a2*d5*dtheta1*m4*cos(theta3 + theta4) + 0.5*a2*d5*dtheta1*m4*cos(2.0*theta2 + theta3 + theta4) + 0.5*a3*d5*dtheta1*m4*cos(theta4);
    C(1,5) = 0;
    C(2,1) = 0.125*dtheta1*m4*width4^2*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.0417*d6^2*dtheta1*m5*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.167*d5^2*dtheta1*m4*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.125*dtheta1*m5*width5^2*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.25*dtheta5*m5*width5^2*sin(theta2 + theta3 + theta4) + 0.125*a2^2*dtheta1*m2*sin(2.0*theta2) + 0.5*a2^2*dtheta1*m3*sin(2.0*theta2) + 0.5*a2^2*dtheta1*m4*sin(2.0*theta2) + 0.5*a2^2*dtheta1*m5*sin(2.0*theta2) + 0.125*a3^2*dtheta1*m3*sin(2.0*theta2 + 2.0*theta3) + 0.5*a3^2*dtheta1*m4*sin(2.0*theta2 + 2.0*theta3) + 0.125*a3^2*dtheta1*m5*sin(2.0*theta2 + 2.0*theta3) + 0.5*a2*a3*dtheta1*m3*sin(2.0*theta2 + theta3) + a2*a3*dtheta1*m4*sin(2.0*theta2 + theta3) + 0.5*a2*a3*dtheta1*m5*sin(2.0*theta2 + theta3) - 0.5*a3*d5*dtheta1*m4*cos(2.0*theta2 + 2.0*theta3 + theta4) - 0.5*a2*d5*dtheta1*m4*cos(2.0*theta2 + theta3 + theta4);
    C(2,2) = 0;
    C(2,3) = -0.5*a2*(2.0*a3*dtheta2*m3*sin(theta3) - 1.0*d5*dtheta3*m4*cos(theta3 + theta4) - 1.0*d5*dtheta4*m4*cos(theta3 + theta4) - 1.0*d5*dtheta4*m5*cos(theta3 + theta4) - 2.0*d5*dtheta2*m4*cos(theta3 + theta4) + 4.0*a3*dtheta2*m4*sin(theta3) + a3*dtheta3*m3*sin(theta3) + 2.0*a3*dtheta2*m5*sin(theta3) + 2.0*a3*dtheta3*m4*sin(theta3) + a3*dtheta3*m5*sin(theta3));
    C(2,4) = 0.25*d5*(4.0*a2*dtheta2*m4*cos(theta3 + theta4) + 2.0*a2*dtheta3*m4*cos(theta3 + theta4) + 2.0*a2*dtheta4*m4*cos(theta3 + theta4) + 2.0*a2*dtheta4*m5*cos(theta3 + theta4) + 4.0*a3*dtheta2*m4*cos(theta4) + 4.0*a3*dtheta3*m4*cos(theta4) + 2.0*a3*dtheta4*m4*cos(theta4) + a3*dtheta4*m5*cos(theta4));
    C(2,5) = 0.25*dtheta1*m5*width5^2*sin(theta2 + theta3 + theta4);
    C(3,1) = 0.125*dtheta1*m4*width4^2*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.0417*d6^2*dtheta1*m5*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.167*d5^2*dtheta1*m4*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.125*dtheta1*m5*width5^2*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.25*dtheta5*m5*width5^2*sin(theta2 + theta3 + theta4) + 0.125*a3^2*dtheta1*m3*sin(2.0*theta2 + 2.0*theta3) + 0.5*a3^2*dtheta1*m4*sin(2.0*theta2 + 2.0*theta3) + 0.125*a3^2*dtheta1*m5*sin(2.0*theta2 + 2.0*theta3) + 0.25*a2*a3*dtheta1*m3*sin(2.0*theta2 + theta3) + 0.5*a2*a3*dtheta1*m4*sin(2.0*theta2 + theta3) + 0.25*a2*a3*dtheta1*m5*sin(2.0*theta2 + theta3) - 0.5*a3*d5*dtheta1*m4*cos(2.0*theta2 + 2.0*theta3 + theta4) - 0.25*a2*d5*dtheta1*m4*cos(theta3 + theta4) - 0.25*a2*d5*dtheta1*m4*cos(2.0*theta2 + theta3 + theta4) + 0.25*a2*a3*dtheta1*m3*sin(theta3) + 0.5*a2*a3*dtheta1*m4*sin(theta3) + 0.25*a2*a3*dtheta1*m5*sin(theta3);
    C(3,2) = 0.25*a2*(2.0*a3*dtheta2*m3*sin(theta3) - 1.0*d5*dtheta3*m4*cos(theta3 + theta4) - 1.0*d5*dtheta4*m4*cos(theta3 + theta4) - 1.0*d5*dtheta4*m5*cos(theta3 + theta4) - 2.0*d5*dtheta2*m4*cos(theta3 + theta4) + 4.0*a3*dtheta2*m4*sin(theta3) + a3*dtheta3*m3*sin(theta3) + 2.0*a3*dtheta2*m5*sin(theta3) + 2.0*a3*dtheta3*m4*sin(theta3) + a3*dtheta3*m5*sin(theta3));
    C(3,3) = -0.25*a2*dtheta2*(a3*m3*sin(theta3) + 2.0*a3*m4*sin(theta3) + a3*m5*sin(theta3) - 1.0*d5*m4*cos(theta3 + theta4));
    C(3,4) = 0.25*d5*(a2*dtheta2*m4*cos(theta3 + theta4) - 1.0*a2*dtheta2*m5*cos(theta3 + theta4) + 4.0*a3*dtheta2*m4*cos(theta4) + 4.0*a3*dtheta3*m4*cos(theta4) + 2.0*a3*dtheta4*m4*cos(theta4) + a3*dtheta4*m5*cos(theta4));
    C(3,5) = 0.25*dtheta1*m5*width5^2*sin(theta2 + theta3 + theta4);
    C(4,1) = 0.125*dtheta1*m4*width4^2*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.0417*d6^2*dtheta1*m5*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.167*d5^2*dtheta1*m4*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.125*dtheta1*m5*width5^2*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.25*dtheta5*m5*width5^2*sin(theta2 + theta3 + theta4) - 0.25*a3*d5*dtheta1*m4*cos(2.0*theta2 + 2.0*theta3 + theta4) - 0.25*a2*d5*dtheta1*m4*cos(theta3 + theta4) - 0.25*a2*d5*dtheta1*m4*cos(2.0*theta2 + theta3 + theta4) - 0.25*a3*d5*dtheta1*m4*cos(theta4);
    C(4,2) = -0.125*d5*(4.0*a2*dtheta2*m4*cos(theta3 + theta4) + 2.0*a2*dtheta3*m4*cos(theta3 + theta4) + 2.0*a2*dtheta4*m4*cos(theta3 + theta4) + 2.0*a2*dtheta4*m5*cos(theta3 + theta4) + 4.0*a3*dtheta2*m4*cos(theta4) + 4.0*a3*dtheta3*m4*cos(theta4) + 2.0*a3*dtheta4*m4*cos(theta4) + a3*dtheta4*m5*cos(theta4));
    C(4,3) = -0.125*d5*(4.0*a3*dtheta2*m4*cos(theta4) - 4.0*a2*dtheta2*m5*cos(theta3 + theta4) - 2.0*a2*dtheta2*m4*cos(theta3 + theta4) + 4.0*a3*dtheta3*m4*cos(theta4) + 2.0*a3*dtheta4*m4*cos(theta4) + a3*dtheta4*m5*cos(theta4));
    C(4,4) = 0.125*d5*(2.0*a2*dtheta2*m4*cos(theta3 + theta4) + 2.0*a2*dtheta2*m5*cos(theta3 + theta4) + 2.0*a3*dtheta2*m4*cos(theta4) + a3*dtheta2*m5*cos(theta4) + 2.0*a3*dtheta3*m4*cos(theta4) + a3*dtheta3*m5*cos(theta4));
    C(4,5) = 0.25*dtheta1*m5*width5^2*sin(theta2 + theta3 + theta4);
    C(5,1) = 0;
    C(5,2) = -0.5*dtheta1*m5*width5^2*sin(theta2 + theta3 + theta4);
    C(5,3) = -0.5*dtheta1*m5*width5^2*sin(theta2 + theta3 + theta4);
    C(5,4) = -0.5*dtheta1*m5*width5^2*sin(theta2 + theta3 + theta4);
    C(5,5) = 0;
end

