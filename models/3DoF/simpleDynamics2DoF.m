clear 

syms L1 L2 m1 m2 m3 rho h w

syms theta1 theta2

syms dtheta1 dtheta2

T0j1 = getZRot(theta1);
T01 = T0j1*getTraslation([L1,0,0]);
T0j2 = T01*getZRot(theta2);    
T02 = T0j2*getTraslation([L2,0,0]);

Jp01(:,1) = cross(T0j1(1:3,3),(T01(1:3,4)-T0j1(1:3,4)));
Jp01(:,2) = zeros(3,1);
Jp01(:,3) = zeros(3,1);

Jo01 = simplify([T0j1(1:3,3) zeros(3,1) zeros(3,1)]);

% I11(1,1) = rho*h*w*a1*(h*h+w*w)/12;
% I11(2,2) = rho*h*w*a1*(h*h+a1*a1)/12;
% I11(3,3) = rho*h*w*a1*(a1*a1+w*w)/12;

% I01 = T01(1:3,1:3)*I11/T01(1:3,1:3);

Jp02(:,1) = cross(T0j1(1:3,3),(T02(1:3,4)-T0j1(1:3,4)));
Jp02(:,2) = cross(T0j2(1:3,3),(T02(1:3,4)-T0j2(1:3,4)));
Jp02(:,3) = zeros(3,1);

Jo02 = [T0j1(1:3,3) T0j2(1:3,3) zeros(3,1)];

% I22(1,1) = rho*h*w*a2*(h*h+w*w)/12;
% I22(2,2) = rho*h*w*a2*(h*h+a2*a2)/12;
% I22(3,3) = rho*h*w*a2*(a2*a2+w*w)/12;

% I02 = T02(1:3,1:3)*I22/T02(1:3,1:3);

B = m1*(Jp01.')*Jp01+m2*(Jp02.')*Jp02;
B = B(1:2,1:2);

syms g;

G0 = [0; -g; 0];

G = simplify(-(m1*G0.'*Jp01 + m2*G0.'*Jp02).');
G = G(1:2);

dG(:,1) = diff(G,theta1);
dG(:,2) = diff(G,theta2);

C(1,:) = [dtheta1 dtheta2]*([diff(B(1,:).',theta1) diff(B(1,:).',theta2)] - 1/2*diff(B(:,:),theta1))*[dtheta1 dtheta2].';
C(2,:) = [dtheta1 dtheta2]*([diff(B(2,:).',theta1) diff(B(2,:).',theta2)] - 1/2*diff(B(:,:),theta2))*[dtheta1 dtheta2].';

B = simplify(B);
G = simplify(G);
dG = simplify(dG);
C = simplify(C);