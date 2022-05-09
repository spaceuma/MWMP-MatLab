function B = getB5(joints_position)

    global d1 a2 a3 d5 d6;
    global m1 m2 m3 m4 m5;
    global width1 width2 width3 width4 width5;

    theta1 = joints_position(1);
    theta2 = joints_position(2);
    theta3 = joints_position(3);
    theta4 = joints_position(4);
    theta5 = joints_position(5);
    
    B = zeros(5,5);
    
    B(1,1) = 0.208*a2^2*m2 + 0.5*a2^2*m3 + 0.5*a2^2*m4 + 0.208*a3^2*m3 + 0.5*a2^2*m5 + 0.5*a3^2*m4 + 0.125*a3^2*m5 + 0.167*d5^2*m4 + 0.0417*d6^2*m5 + 0.5*m1*width1^2 + 0.25*m2*width2^2 + 0.25*m3*width3^2 + 0.375*m4*width4^2 + 0.375*m5*width5^2 + 0.125*a2^2*m2*cos(2.0*theta2) + 0.5*a2^2*m3*cos(2.0*theta2) + 0.5*a2^2*m4*cos(2.0*theta2) + 0.5*a2^2*m5*cos(2.0*theta2) + 0.125*a3^2*m3*cos(2.0*theta2 + 2.0*theta3) + 0.5*a3^2*m4*cos(2.0*theta2 + 2.0*theta3) + 0.125*a3^2*m5*cos(2.0*theta2 + 2.0*theta3) - 0.167*d5^2*m4*cos(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 0.0417*d6^2*m5*cos(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.125*m4*width4^2*cos(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.125*m5*width5^2*cos(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.5*a3*d5*m4*sin(2.0*theta2 + 2.0*theta3 + theta4) + 0.5*a2*d5*m4*sin(theta3 + theta4) + 0.5*a2*d5*m4*sin(2.0*theta2 + theta3 + theta4) + 0.5*a2*a3*m3*cos(theta3) + a2*a3*m4*cos(theta3) + 0.5*a2*a3*m5*cos(theta3) + 0.5*a3*d5*m4*sin(theta4) + 0.5*a2*a3*m3*cos(2.0*theta2 + theta3) + a2*a3*m4*cos(2.0*theta2 + theta3) + 0.5*a2*a3*m5*cos(2.0*theta2 + theta3);
    B(1,2) = 0;
    B(1,3) = 0;
    B(1,4) = 0;
    B(1,5) = 0.5*m5*width5^2*cos(theta2 + theta3 + theta4);
    B(2,1) = 0;
    B(2,2) = 0.25*a2^2*m2 + a2^2*m3 + a2^2*m4 + 0.25*a3^2*m3 + a2^2*m5 + a3^2*m4 + 0.25*a3^2*m5 + 0.333*d5^2*m4 + 0.0833*d6^2*m5 + 0.5*m2*width2^2 + 0.5*m3*width3^2 + 0.25*m4*width4^2 + 0.25*m5*width5^2 + a2*d5*m4*sin(theta3 + theta4) + a2*a3*m3*cos(theta3) + 2.0*a2*a3*m4*cos(theta3) + a2*a3*m5*cos(theta3) + a3*d5*m4*sin(theta4);
    B(2,3) = 0.25*a3^2*m3 + a3^2*m4 + 0.25*a3^2*m5 + 0.333*d5^2*m4 + 0.0833*d6^2*m5 + 0.5*m3*width3^2 + 0.25*m4*width4^2 + 0.25*m5*width5^2 + 0.5*a2*d5*m4*sin(theta3 + theta4) + 0.5*a2*a3*m3*cos(theta3) + a2*a3*m4*cos(theta3) + 0.5*a2*a3*m5*cos(theta3) + a3*d5*m4*sin(theta4);
    B(2,4) = 0.333*d5^2*m4 + 0.0833*d6^2*m5 + 0.25*m4*width4^2 + 0.25*m5*width5^2 + 0.5*a2*d5*m4*sin(theta3 + theta4) + 0.5*a2*d5*m5*sin(theta3 + theta4) + 0.5*a3*d5*m4*sin(theta4) + 0.25*a3*d5*m5*sin(theta4);
    B(2,5) = 0;
    B(3,1) = 0;
    B(3,2) = 0.25*a3^2*m3 + a3^2*m4 + 0.25*a3^2*m5 + 0.333*d5^2*m4 + 0.0833*d6^2*m5 + 0.5*m3*width3^2 + 0.25*m4*width4^2 + 0.25*m5*width5^2 + 0.5*a2*d5*m4*sin(theta3 + theta4) + 0.5*a2*a3*m3*cos(theta3) + a2*a3*m4*cos(theta3) + 0.5*a2*a3*m5*cos(theta3) + a3*d5*m4*sin(theta4);
    B(3,3) = 0.25*a3^2*m3 + a3^2*m4 + 0.25*a3^2*m5 + 0.333*d5^2*m4 + 0.0833*d6^2*m5 + 0.5*m3*width3^2 + 0.25*m4*width4^2 + 0.25*m5*width5^2 + a3*d5*m4*sin(theta4);
    B(3,4) = 0.333*d5^2*m4 + 0.0833*d6^2*m5 + 0.25*m4*width4^2 + 0.25*m5*width5^2 + 0.5*a3*d5*m4*sin(theta4) + 0.25*a3*d5*m5*sin(theta4);
    B(3,5) = 0;
    B(4,1) = 0;
    B(4,2) = 0.333*d5^2*m4 + 0.0833*d6^2*m5 + 0.25*m4*width4^2 + 0.25*m5*width5^2 + 0.5*a2*d5*m4*sin(theta3 + theta4) + 0.5*a2*d5*m5*sin(theta3 + theta4) + 0.5*a3*d5*m4*sin(theta4) + 0.25*a3*d5*m5*sin(theta4);
    B(4,3) = 0.333*d5^2*m4 + 0.0833*d6^2*m5 + 0.25*m4*width4^2 + 0.25*m5*width5^2 + 0.5*a3*d5*m4*sin(theta4) + 0.25*a3*d5*m5*sin(theta4);
    B(4,4) = 0.333*d5^2*m4 + 0.25*d5^2*m5 + 0.0833*d6^2*m5 + 0.25*m4*width4^2 + 0.25*m5*width5^2;
    B(4,5) = 0;
    B(5,1) = 0.5*m5*width5^2*cos(theta2 + theta3 + theta4);
    B(5,2) = 0;
    B(5,3) = 0;
    B(5,4) = 0;
    B(5,5) = 0.5*m5*width5^2;
end

