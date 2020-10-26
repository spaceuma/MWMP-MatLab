function [G, dG] = getG3(theta1, theta2, theta3)
    G = zeros(3,1);
    dG = zeros(3,3);
    
    G(1,1) = 0;
    G(2,1) = - 31.985*cos(theta2 + theta3) - 103.42*cos(theta2);
    G(3,1) = -31.985*cos(theta2 + theta3);
    
    dG(1,1) = 0;
    dG(1,2) = 0;
    dG(1,3) = 0;
    dG(2,1) = 0;
    dG(2,2) = 31.985*sin(theta2 + theta3) + 103.42*sin(theta2);
    dG(2,3) = 31.985*sin(theta2 + theta3);
    dG(3,1) = 0;
    dG(3,2) = 31.985*sin(theta2 + theta3);
    dG(3,3) = 31.985*sin(theta2 + theta3);
end

