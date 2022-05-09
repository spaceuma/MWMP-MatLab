function G = getG3(theta1, theta2, theta3)
    global m1 m2 m3;
    global a2 d4;

    G = zeros(3,1);
    
    G(1,1) = 0;
    G(2,1) = -(a2*m2*cos(theta2) + 2*a2*m3*cos(theta2) + d4*m3*cos(theta2 + theta3))/2;
    G(3,1) = -(d4*m3*cos(theta2 + theta3))/2;
end

