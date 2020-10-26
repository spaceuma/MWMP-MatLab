function C = getC3(theta1, theta2, theta3, dtheta1, dtheta2, dtheta3)
    C = zeros(3,3);
    
    C(1,1) = 0;
    C(1,2) = -1.1102e-22*dtheta1*(4.317e+22*sin(2.0*theta2 + theta3) + 6.1674e+22*sin(2.0*theta2) + 1.3215e+22*sin(theta2 + theta3) + 4.2732e+22*sin(theta2) + 6.821e+21*sin(2.0*theta2 + 2.0*theta3));
    C(1,3) = -1.1102e-22*dtheta1*(2.1585e+22*sin(2.0*theta2 + theta3) + 1.3215e+22*sin(theta2 + theta3) + 2.1585e+22*sin(theta3) + 6.821e+21*sin(2.0*theta2 + 2.0*theta3));
    C(2,1) = 5.5511e-23*dtheta1*(4.317e+22*sin(2.0*theta2 + theta3) + 6.1674e+22*sin(2.0*theta2) + 1.3215e+22*sin(theta2 + theta3) + 4.2732e+22*sin(theta2) + 6.821e+21*sin(2.0*theta2 + 2.0*theta3));
    C(2,2) = 0;
    C(2,3) = -2.3964*sin(theta3)*(2.0*dtheta2 + dtheta3);
    C(3,1) = 5.5511e-23*dtheta1*(2.1585e+22*sin(2.0*theta2 + theta3) + 1.3215e+22*sin(theta2 + theta3) + 2.1585e+22*sin(theta3) + 6.821e+21*sin(2.0*theta2 + 2.0*theta3));
    C(3,2) = 1.1982*sin(theta3)*(2.0*dtheta2 + dtheta3);
    C(3,3) = -1.1982*dtheta2*sin(theta3);
end

