function [TB0, TB1, TB2, TB3, TB4, TB5] = direct5(manipulatorJoints)
    % Modelo cinematico directo
    global d1;
    global a2;
    global a3;
    global d5;
    global d6;

    
    theta1 = manipulatorJoints(1);
    theta2 = manipulatorJoints(2);
    theta3 = manipulatorJoints(3);
    theta4 = manipulatorJoints(4);
    theta5 = manipulatorJoints(5);


    TB0 = getTraslation([0,0,d1]);
    T01 = getZRot(theta1)*getXRot(-pi/2);
    T12 = getZRot(theta2)*getTraslation([a2,0,0]);
    T23 = getZRot(theta3)*getTraslation([a3,0,0]);
    T34 = getZRot(theta4)*getXRot(pi/2);
    T45 = getTraslation([0,0,d5+d6])*getZRot(theta5);

    TB1 = TB0*T01;
    TB2 = TB1*T12;
    TB3 = TB2*T23;
    TB4 = TB3*T34;
    TB5 = TB4*T45;
end