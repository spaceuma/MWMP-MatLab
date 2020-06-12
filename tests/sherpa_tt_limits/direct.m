function [TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(manipulatorJoints)
    % Modelo cinematico directo
    global d0;
    global a1;
    global a2;
    global c2;
    global a3;
    global d4;
    global d6;
    
    theta1 = manipulatorJoints(1);
    theta2 = manipulatorJoints(2);
    theta3 = manipulatorJoints(3) + pi/2;
    theta4 = manipulatorJoints(4);
    theta5 = manipulatorJoints(5);
    theta6 = manipulatorJoints(6);

    TB0 = getTraslation([0,0,d0]);
    T01 = getZRot(theta1)*getTraslation([a1,0,0])*getXRot(-pi/2);
    T12 = getZRot(theta2)*getTraslation([a2,c2,0]);
    T23 = getZRot(theta3)*getTraslation([-a3,0,0])*getXRot(pi/2);
    T34 = getTraslation([0,0,d4])*getZRot(theta4)*getXRot(-pi/2);
    T45 = getZRot(theta5)*getXRot(pi/2);
    T56 = getTraslation([0,0,d6])*getZRot(theta6);
    
    

    TB1 = TB0*T01;
    TB2 = TB1*T12;
    TB3 = TB2*T23;
    TB4 = TB3*T34;
    TB5 = TB4*T45;
    TB6 = TB5*T56;
end