% Computing slipping conditions
syms theta1 theta2 theta3 theta4 w1 w2 w3 w4 wz1 wz2 wz3 wz4 ws1 ws2 ws3 ws4 R dfx dfy;


% R = 0.2;
% df = 1;

J1 = [-R*sin(theta1) dfy  dfy;
      +R*cos(theta1) -dfx dfx;
      0              1    -1];

J2 = [-R*sin(theta2) dfy  dfy;
      +R*cos(theta2) dfx -dfx;
      0                1  -1];

J3 = [-R*sin(theta3) -dfy -dfy;
      +R*cos(theta3)  dfx -dfx;
      0                 1  -1];
  
J4 = [-R*sin(theta4) -dfy -dfy;
      +R*cos(theta4) -dfx  dfx;
      0                 1  -1];
  
A=[eye(3);eye(3);eye(3);eye(3)];
B = [J1         zeros(3,9);
     zeros(3,3) J2 zeros(3,6);
     zeros(3,6)    J3 zeros(3,3)
     zeros(3,9)       J4];
 
qp = [w1; wz1; ws1; w2; wz2; ws2; w3; wz3; ws3; w4; wz4; ws4];
 
pretty((A'*A)\A'*B*qp)

pretty(simplify(omega(A)*B*qp))