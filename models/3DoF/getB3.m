function B = getB3(theta1, theta2, theta3)
    B = zeros(3,3);
    
    B(1,1) = 2.3964*cos(2.0*theta2 + theta3) + 3.4236*cos(2.0*theta2) + 1.4672*cos(theta2 + theta3) + 4.7442*cos(theta2) + 2.3964*cos(theta3) + 0.37864*cos(2.0*theta2 + 2.0*theta3) + 5.2292;
    B(1,2) = 0;
    B(1,3) = 0;
    B(2,1) = 0;
    B(2,2) = 4.7928*cos(theta3) + 8.3763;
    B(2,3) = 2.3964*cos(theta3) + 1.5126;
    B(3,1) = 0;
    B(3,2) = 2.3964*cos(theta3) + 1.5126;
    B(3,3) = 1.5126;



end

