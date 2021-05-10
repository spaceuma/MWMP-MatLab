function [TB0, TB1, TB2, TB3] = direct3(manipulatorJoints)
    % Modelo cinematico directo
    global d0;
    global a1;
    global a2;
    global d4;

    
    theta1 = manipulatorJoints(1);
    theta2 = manipulatorJoints(2);
    theta3 = manipulatorJoints(3) + pi/2;

    TB0 = getTraslation([0,0,d0]);
    T01 = getZRot(theta1)*getTraslation([a1,0,0])*getXRot(-pi/2);
    T12 = getZRot(theta2)*getTraslation([a2,0,0]);
    T23 = getZRot(theta3)*getXRot(pi/2)*getTraslation([0,0,d4]);

    TB1 = TB0*T01;
    TB2 = TB1*T12;
    TB3 = TB2*T23;
end