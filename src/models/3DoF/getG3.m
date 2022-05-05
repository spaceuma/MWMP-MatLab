function [G, dG] = getG3(theta1, theta2, theta3, g)
    G = zeros(3,1);
    dG = zeros(3,3);
    
    G(1,1) = 0;
    G(2,1) = - 30.681*cos(theta2 + theta3) - 97.34*cos(theta2);
    G(3,1) = -30.681*cos(theta2 + theta3);

    G = G*g/9.81;
    
    dG(1,1) = 0;
    dG(1,2) = 0;
    dG(1,3) = 0;
    dG(2,1) = 0;
    dG(2,2) = 30.681*sin(theta2 + theta3) + 97.34*sin(theta2);
    dG(2,3) = 30.681*sin(theta2 + theta3);
    dG(3,1) = 0;
    dG(3,2) = 30.681*sin(theta2 + theta3);
    dG(3,3) = 30.681*sin(theta2 + theta3);

    dG = dG*g/9.81;
end

