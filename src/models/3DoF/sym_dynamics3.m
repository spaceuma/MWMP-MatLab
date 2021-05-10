%     global a1;
%     global a2;
%     global d4;
%     
%     global h;
%     global w;
%     
%     global rho;
%     
%     m1 = h*w*a1*rho;
%     m2 = h*w*a2*rho;
%     m3 = h*w*d4*rho;
syms a1 a2 d4 m1 m2 m3 rho h w g

syms theta1 theta2 theta3

syms dtheta1 dtheta2 dtheta3

T0j1 = getZRot(theta1);
T01 = T0j1*getTraslation([a1/2,0,0]);
T0j2 = T01*getTraslation([a1/2,0,0])*getXRot(-pi/2)*getZRot(theta2);    
T02 = T0j2*getTraslation([a2/2,0,0]);
T0j3 = T02*getTraslation([a2/2,0,0])*getZRot(theta3+pi/2);
T03 = T0j3*getXRot(pi/2)*getTraslation([0,0,d4/2]);

Jp01(:,1) = cross(T0j1(1:3,3),(T01(1:3,4)-T0j1(1:3,4)));
Jp01(:,2) = zeros(3,1);
Jp01(:,3) = zeros(3,1);

Jo01 = simplify([T0j1(1:3,3) zeros(3,1) zeros(3,1)]);

I11(1,1) = m1*(h*h+w*w)/12;
I11(2,2) = m1*(h*h+a1*a1)/12;
I11(3,3) = m1*(a1*a1+w*w)/12;
I11 = 0;

I01 = T01(1:3,1:3)*I11/T01(1:3,1:3);

Jp02(:,1) = cross(T0j1(1:3,3),(T02(1:3,4)-T0j1(1:3,4)));
Jp02(:,2) = cross(T0j2(1:3,3),(T02(1:3,4)-T0j2(1:3,4)));
Jp02(:,3) = zeros(3,1);

Jo02 = [T0j1(1:3,3) T0j2(1:3,3) zeros(3,1)];

I22(1,1) = m2*(h*h+w*w)/12;
I22(2,2) = m2*(h*h+a2*a2)/12;
I22(3,3) = m2*(a2*a2+w*w)/12;
I22 = 0;

I02 = T02(1:3,1:3)*I22/T02(1:3,1:3);

Jp03(:,1) = cross(T0j1(1:3,3),(T03(1:3,4)-T0j1(1:3,4)));
Jp03(:,2) = cross(T0j2(1:3,3),(T03(1:3,4)-T0j2(1:3,4)));
Jp03(:,3) = cross(T0j3(1:3,3),(T03(1:3,4)-T0j3(1:3,4)));

Jo03 = [T0j1(1:3,3) T0j2(1:3,3) T0j3(1:3,3)];   

I33(1,1) = m3*(h*h+w*w)/12;
I33(2,2) = m3*(h*h+d4*d4)/12;
I33(3,3) = m3*(d4*d4+w*w)/12;
I33 = 0;

I03 = T03(1:3,1:3)*I33/T03(1:3,1:3);

B = m1*(Jp01.')*Jp01+(Jo01.')*I01*Jo01 + ...
    m2*(Jp02.')*Jp02+(Jo02.')*I02*Jo02 + ...
    m3*(Jp03.')*Jp03+(Jo03.')*I03*Jo03;

G0 = [0; 0; -g];

G = simplify(-(m1*G0.'*Jp01 + m2*G0.'*Jp02 + m3*G0.'*Jp03).');

dG(:,1) = diff(G,theta1);
dG(:,2) = diff(G,theta2);
dG(:,3) = diff(G,theta3);    

C(1,:) = [dtheta1 dtheta2 dtheta3]*([diff(B(1,:).',theta1) diff(B(1,:).',theta2) diff(B(1,:).',theta3)] - 1/2*diff(B(:,:),theta1));
C(2,:) = [dtheta1 dtheta2 dtheta3]*([diff(B(2,:).',theta1) diff(B(2,:).',theta2) diff(B(2,:).',theta3)] - 1/2*diff(B(:,:),theta2));
C(3,:) = [dtheta1 dtheta2 dtheta3]*([diff(B(3,:).',theta1) diff(B(3,:).',theta2) diff(B(3,:).',theta3)] - 1/2*diff(B(:,:),theta3));

B = simplify(B);
G = simplify(G);
dG = simplify(dG);
C = simplify(C);

%     vpa(B,3)
%     vpa(G,3)
%     vpa(C,3)


