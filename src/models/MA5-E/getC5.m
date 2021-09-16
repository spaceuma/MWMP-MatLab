function C = getC5(joints_position, joints_speed)

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
    C(1,2) = 0.00293*dtheta1*cos(2.0*theta2 + 2.0*theta3 + theta4) - 0.058*dtheta1*sin(2.0*theta2 + theta3) - 0.0637*dtheta1*sin(2.0*theta2) + 0.00343*dtheta1*cos(2.0*theta2 + theta3 + theta4) - 0.017*dtheta1*sin(2.0*theta2 + 2.0*theta3) + 6.35e-4*dtheta1*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 4.0e-5*dtheta5*sin(theta2 + theta3 + theta4);
    C(1,3) = 0.00293*dtheta1*cos(2.0*theta2 + 2.0*theta3 + theta4) - 0.029*dtheta1*sin(2.0*theta2 + theta3) + 0.00171*dtheta1*cos(theta3 + theta4) + 0.00171*dtheta1*cos(2.0*theta2 + theta3 + theta4) - 0.029*dtheta1*sin(theta3) - 0.017*dtheta1*sin(2.0*theta2 + 2.0*theta3) + 6.35e-4*dtheta1*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 4.0e-5*dtheta5*sin(theta2 + theta3 + theta4);
    C(1,4) = 0.00147*dtheta1*cos(2.0*theta2 + 2.0*theta3 + theta4) + 0.00171*dtheta1*cos(theta3 + theta4) + 0.00171*dtheta1*cos(2.0*theta2 + theta3 + theta4) + 0.00147*dtheta1*cos(theta4) + 6.35e-4*dtheta1*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 4.0e-5*dtheta5*sin(theta2 + theta3 + theta4);
    C(1,5) = 0;
    C(2,1) = 0.029*dtheta1*sin(2.0*theta2 + theta3) - 0.00147*dtheta1*cos(2.0*theta2 + 2.0*theta3 + theta4) + 0.0318*dtheta1*sin(2.0*theta2) - 0.00171*dtheta1*cos(2.0*theta2 + theta3 + theta4) + 0.00852*dtheta1*sin(2.0*theta2 + 2.0*theta3) - 3.17e-4*dtheta1*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 2.0e-5*dtheta5*sin(theta2 + theta3 + theta4);
    C(2,2) = 0;
    C(2,3) = 0.00343*dtheta2*cos(theta3 + theta4) + 0.00171*dtheta3*cos(theta3 + theta4) + 0.00286*dtheta4*cos(theta3 + theta4) - 0.058*dtheta2*sin(theta3) - 0.029*dtheta3*sin(theta3);
    C(2,4) = 0.00343*dtheta2*cos(theta3 + theta4) + 0.00171*dtheta3*cos(theta3 + theta4) + 0.00286*dtheta4*cos(theta3 + theta4) + 0.00293*dtheta2*cos(theta4) + 0.00293*dtheta3*cos(theta4) + 0.00195*dtheta4*cos(theta4);
    C(2,5) = 2.0e-5*dtheta1*sin(theta2 + theta3 + theta4);
    C(3,1) = 0.0145*dtheta1*sin(2.0*theta2 + theta3) - 0.00147*dtheta1*cos(2.0*theta2 + 2.0*theta3 + theta4) - 8.57e-4*dtheta1*cos(theta3 + theta4) - 8.57e-4*dtheta1*cos(2.0*theta2 + theta3 + theta4) + 0.0145*dtheta1*sin(theta3) + 0.00852*dtheta1*sin(2.0*theta2 + 2.0*theta3) - 3.17e-4*dtheta1*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 2.0e-5*dtheta5*sin(theta2 + theta3 + theta4);
    C(3,2) = 0.029*dtheta2*sin(theta3) - 8.57e-4*dtheta3*cos(theta3 + theta4) - 0.00143*dtheta4*cos(theta3 + theta4) - 0.00171*dtheta2*cos(theta3 + theta4) + 0.0145*dtheta3*sin(theta3);
    C(3,3) = 2.58e-6*dtheta2*(333.0*cos(theta3 + theta4) - 5630.0*sin(theta3));
    C(3,4) = 2.86e-4*dtheta2*cos(theta3 + theta4) + 0.00293*dtheta2*cos(theta4) + 0.00293*dtheta3*cos(theta4) + 0.00195*dtheta4*cos(theta4);
    C(3,5) = 2.0e-5*dtheta1*sin(theta2 + theta3 + theta4);
    C(4,1) = 2.0e-5*dtheta5*sin(theta2 + theta3 + theta4) - 8.57e-4*dtheta1*cos(theta3 + theta4) - 8.57e-4*dtheta1*cos(2.0*theta2 + theta3 + theta4) - 7.33e-4*dtheta1*cos(theta4) - 3.17e-4*dtheta1*sin(2.0*theta2 + 2.0*theta3 + 2.0*theta4) - 7.33e-4*dtheta1*cos(2.0*theta2 + 2.0*theta3 + theta4);
    C(4,2) = - 0.00171*dtheta2*cos(theta3 + theta4) - 8.57e-4*dtheta3*cos(theta3 + theta4) - 0.00143*dtheta4*cos(theta3 + theta4) - 0.00147*dtheta2*cos(theta4) - 0.00147*dtheta3*cos(theta4) - 9.77e-4*dtheta4*cos(theta4);
    C(4,3) = 0.002*dtheta2*cos(theta3 + theta4) - 0.00147*dtheta2*cos(theta4) - 0.00147*dtheta3*cos(theta4) - 9.77e-4*dtheta4*cos(theta4);
    C(4,4) = 0.00143*dtheta2*cos(theta3 + theta4) + 9.77e-4*dtheta2*cos(theta4) + 9.77e-4*dtheta3*cos(theta4);
    C(4,5) = 2.0e-5*dtheta1*sin(theta2 + theta3 + theta4);
    C(5,1) = 0;
    C(5,2) = -4.0e-5*dtheta1*sin(theta2 + theta3 + theta4);
    C(5,3) = -4.0e-5*dtheta1*sin(theta2 + theta3 + theta4);
    C(5,4) = -4.0e-5*dtheta1*sin(theta2 + theta3 + theta4);
    C(5,5) = 0;
end

