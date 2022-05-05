function B = getB3(theta1, theta2, theta3)
    B = zeros(3,3);
    
    B(1,1) = 2.2987*cos(2.0*theta2 + theta3) + 3.2376*cos(2.0*theta2) + 1.4074*cos(theta2 + theta3) + 4.4651*cos(theta2) + 2.2987*cos(theta3) + 0.36321*cos(2.0*theta2 + 2.0*theta3) + 4.9443;
    B(1,2) = 0;
    B(1,3) = 0;
    B(2,1) = 0;
    B(2,2) = 4.5974*cos(theta3) + 7.9411;
    B(2,3) = 2.2987*cos(theta3) + 1.4509;
    B(3,1) = 0;
    B(3,2) = 2.2987*cos(theta3) + 1.4509;
    B(3,3) = 1.4509;



end

