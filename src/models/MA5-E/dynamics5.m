function [B, G] = dynamics5(manipulatorJoints)
    % Inertia Matrix
    global d1;
    global a2;
    global a3;
    global d5;
    global d6;
    
    global width1;
    global width2;
    global width3;
    global width4;
    global width5;
        
    global m1;
    global m2;
    global m3;
    global m4;
    global m5;
    
    theta1 = manipulatorJoints(1);
    theta2 = manipulatorJoints(2);
    theta3 = manipulatorJoints(3);
    theta4 = manipulatorJoints(4);
    theta5 = manipulatorJoints(5);
        
    T0j1 = getZRot(theta1);
    T01 = T0j1*getTraslation([0,0,d1/2]);
    T0j2 = T01*getTraslation([0,0,d1/2])*getXRot(-pi/2)*getZRot(theta2);
    T02 = T0j2*getTraslation([a2/2,0,0]);
    T0j3 = T02*getTraslation([a2/2,0,0])*getZRot(theta3);
    T03 = T0j3*getTraslation([a3/2,0,0]);
    T0j4 = T03*getTraslation([a3/2,0,0])*getZRot(theta4)*getXRot(pi/2);
    T04 = T0j4*getTraslation([0,0,d5/2]);
    T0j5 = T04*getTraslation([0,0,d5/2])*getZRot(theta5);
    T05 = T0j5*getTraslation([0,0,d6/2]);

    Jp01 = zeros(3,5);
    Jp01(:,1) = cross(T0j1(1:3,3),(T01(1:3,4)-T0j1(1:3,4)));
    
    Jo01 = [T0j1(1:3,3) zeros(3,1) zeros(3,1) zeros(3,1) zeros(3,1)];
    
    I11 = zeros(3,3);
    I11(1,1) = m1*width1^2/2 + m1*d1^2/12;
    I11(2,2) = m1*width1^2/2 + m1*d1^2/12;
    I11(3,3) = m1*width1^2/2;
    
    I01 = T01(1:3,1:3)*I11/T01(1:3,1:3);
    
    Jp02 = zeros(3,5);
    Jp02(:,1) = cross(T0j1(1:3,3),(T02(1:3,4)-T0j1(1:3,4)));
    Jp02(:,2) = cross(T0j2(1:3,3),(T02(1:3,4)-T0j2(1:3,4)));
    
    Jo02 = [T0j1(1:3,3) T0j2(1:3,3) zeros(3,1) zeros(3,1) zeros(3,1)];
    
    I22 = zeros(3,3);
    I22(1,1) = m2*width2^2/2 + m2*a2^2/12;
    I22(2,2) = m2*width2^2/2 + m2*a2^2/12;
    I22(3,3) = m2*width2^2/2;
    
    I02 = T02(1:3,1:3)*I22/T02(1:3,1:3);
    
    Jp03 = zeros(3,5);
    Jp03(:,1) = cross(T0j1(1:3,3),(T03(1:3,4)-T0j1(1:3,4)));
    Jp03(:,2) = cross(T0j2(1:3,3),(T03(1:3,4)-T0j2(1:3,4)));
    Jp03(:,3) = cross(T0j3(1:3,3),(T03(1:3,4)-T0j3(1:3,4)));
    
    Jo03 = [T0j1(1:3,3) T0j2(1:3,3) T0j3(1:3,3) zeros(3,1) zeros(3,1)];   
    
    I33 = zeros(3,3);
    I33(1,1) = m3*width3^2/2 + m3*a3^2/12;
    I33(2,2) = m3*width3^2/2 + m3*a3^2/12;
    I33(3,3) = m3*width3^2/2;
    
    I03 = T03(1:3,1:3)*I33/T03(1:3,1:3);
    
    Jp04 = zeros(3,5);
    Jp04(:,1) = cross(T0j1(1:3,3),(T04(1:3,4)-T0j1(1:3,4)));
    Jp04(:,2) = cross(T0j2(1:3,3),(T04(1:3,4)-T0j2(1:3,4)));
    Jp04(:,3) = cross(T0j3(1:3,3),(T04(1:3,4)-T0j3(1:3,4)));
    Jp04(:,4) = cross(T0j4(1:3,3),(T04(1:3,4)-T0j4(1:3,4)));

    Jo04 = [T0j1(1:3,3) T0j2(1:3,3) T0j3(1:3,3) T0j4(1:3,3) zeros(3,1)];   
    
    I44 = zeros(3,3);
    I44(1,1) = m4*width4^2/2 + m4*d5^2/12;
    I44(2,2) = m4*width4^2/2 + m4*d5^2/12;
    I44(3,3) = m4*width4^2/2;
    
    I04 = T04(1:3,1:3)*I44/T04(1:3,1:3);
        
    Jp05 = zeros(3,5);
    Jp05(:,1) = cross(T0j1(1:3,3),(T03(1:3,4)-T0j1(1:3,4)));
    Jp05(:,2) = cross(T0j2(1:3,3),(T03(1:3,4)-T0j2(1:3,4)));
    Jp05(:,3) = cross(T0j3(1:3,3),(T03(1:3,4)-T0j3(1:3,4)));
    Jp05(:,4) = cross(T0j4(1:3,3),(T04(1:3,4)-T0j4(1:3,4)));
    Jp05(:,5) = cross(T0j5(1:3,3),(T05(1:3,4)-T0j5(1:3,4)));

    
    Jo05 = [T0j1(1:3,3) T0j2(1:3,3) T0j3(1:3,3) T0j4(1:3,3) T0j5(1:3,3)];   
    
    I55 = zeros(3,3);
    I55(1,1) = m5*width5^2/2 + m5*d6^2/12;
    I55(2,2) = m5*width5^2/2 + m5*d6^2/12;
    I55(3,3) = m5*width5^2/2;
    
    I05 = T05(1:3,1:3)*I55/T05(1:3,1:3);
    
    B = m1*(Jp01.')*Jp01+(Jo01.')*I01*Jo01 + ...
        m2*(Jp02.')*Jp02+(Jo02.')*I02*Jo02 + ...
        m3*(Jp03.')*Jp03+(Jo03.')*I03*Jo03 + ...
        m4*(Jp04.')*Jp04+(Jo04.')*I04*Jo04 + ...
        m5*(Jp05.')*Jp05+(Jo05.')*I05*Jo05;
    
    G0 = [0 0 -9.81];
    
    G = -(m1*G0*Jp01 + m2*G0*Jp02 + m3*G0*Jp03 + m4*G0*Jp04 + m5*G0*Jp05).';
    
end

