function B = getB5(joints_position)

    theta1 = joints_position(1);
    theta2 = joints_position(2);
    theta3 = joints_position(3);
    theta4 = joints_position(4);
    theta5 = joints_position(5);
    
    B = zeros(5,5);
    
    B(1,1) = 0.029*cos(2.0*theta2 + theta3) + 0.00147*sin(2.0*theta2 + 2.0*theta3 + theta4) + 0.0318*cos(2.0*theta2) + 0.00171*sin(theta3 + theta4) + 0.00171*sin(2.0*theta2 + theta3 + theta4) + 0.029*cos(theta3) + 0.00147*sin(theta4) + 0.00852*cos(2.0*theta2 + 2.0*theta3) - 3.17e-4*cos(2.0*theta2 + 2.0*theta3 + 2.0*theta4) + 0.047;
    B(1,2) = 0;
    B(1,3) = 0;
    B(1,4) = 0;
    B(1,5) = 4.0e-5*cos(theta2 + theta3 + theta4);
    B(2,1) = 0;
    B(2,2) = 0.00343*sin(theta3 + theta4) + 0.058*cos(theta3) + 0.00293*sin(theta4) + 0.0821;
    B(2,3) = 0.00171*sin(theta3 + theta4) + 0.029*cos(theta3) + 0.00293*sin(theta4) + 0.0182;
    B(2,4) = 0.00286*sin(theta3 + theta4) + 0.00195*sin(theta4) + 0.00105;
    B(2,5) = 0;
    B(3,1) = 0;
    B(3,2) = 0.00171*sin(theta3 + theta4) + 0.029*cos(theta3) + 0.00293*sin(theta4) + 0.0182;
    B(3,3) = 0.00293*sin(theta4) + 0.0182;
    B(3,4) = 0.00195*sin(theta4) + 0.00105;
    B(3,5) = 0;
    B(4,1) = 0;
    B(4,2) = 0.00286*sin(theta3 + theta4) + 0.00195*sin(theta4) + 0.00105;
    B(4,3) = 0.00195*sin(theta4) + 0.00105;
    B(4,4) = 0.0012;
    B(4,5) = 0;
    B(5,1) = 4.0e-5*cos(theta2 + theta3 + theta4);
    B(5,2) = 0;
    B(5,3) = 0;
    B(5,4) = 0;
    B(5,5) = 4.0e-5;
end

