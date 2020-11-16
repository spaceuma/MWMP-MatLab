function B = getB3(theta1, theta2, theta3)
    B = zeros(3,3);
    
    B(1,1) = 2.2987*cos(2.0*theta2 + theta3) + 3.0388*cos(2.0*theta2) + 1.4074*cos(theta2 + theta3) + 4.4651*cos(theta2) + 2.2987*cos(theta3) + 0.5434*cos(2.0*theta2 + 2.0*theta3) + 4.5314;
    B(1,2) = 0;
    B(1,3) = 0;
    B(2,1) = 0;
    B(2,2) = 4.5974*cos(theta3) + 7.1643;
    B(2,3) = 2.2987*cos(theta3) + 1.0868;
    B(3,1) = 0;
    B(3,2) = 2.2987*cos(theta3) + 1.0868;
    B(3,3) = 1.0868;



end

