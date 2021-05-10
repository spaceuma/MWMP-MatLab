function C = getC3(theta1, theta2, theta3, dtheta1, dtheta2, dtheta3)
    C = zeros(3,3);
    
    C(1,1) = 0;
    C(1,2) = -0.25*dtheta1*(18.39*sin(2.0*theta2 + theta3) + 24.31*sin(2.0*theta2) + 5.6295*sin(theta2 + theta3) + 17.86*sin(theta2) + 4.3472*sin(2.0*theta2 + 2.0*theta3));
    C(1,3) = -1.5638*dtheta1*(1.47*sin(2.0*theta2 + theta3) + 0.9*sin(theta2 + theta3) + 1.47*sin(theta3) + 0.695*sin(2.0*theta2 + 2.0*theta3));
    C(2,1) = 0.125*dtheta1*(18.39*sin(2.0*theta2 + theta3) + 24.31*sin(2.0*theta2) + 5.6295*sin(theta2 + theta3) + 17.86*sin(theta2) + 4.3472*sin(2.0*theta2 + 2.0*theta3));
    C(2,2) = 0;
    C(2,3) = -2.2987*sin(theta3)*(2.0*dtheta2 + dtheta3);
    C(3,1) = 0.78188*dtheta1*(1.47*sin(2.0*theta2 + theta3) + 0.9*sin(theta2 + theta3) + 1.47*sin(theta3) + 0.695*sin(2.0*theta2 + 2.0*theta3));
    C(3,2) = 1.1494*sin(theta3)*(2.0*dtheta2 + dtheta3);
    C(3,3) = -1.1494*dtheta2*sin(theta3);
end

