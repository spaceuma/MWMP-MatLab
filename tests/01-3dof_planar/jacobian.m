% Gonzalo J. Paz Delgado
% 12/11/2019
%% Apartado 1
syms q1 q2 q3;
q = [q1 q2 q3];
J = jacobiano3(q);

%% Apartado 2
q1 = 120*pi/180;
q2 = 240*pi/180;
q3 = 270*pi/180;

q = [q1 q2 q3];

qp = [5*pi/180;
10*pi/180;
20*pi/180];

v = jacobiano3(q)*qp

%% Apartado 3

A = [0.3; 0.25];
B = [0.3; 0.05];
phi = 0;

vAB = [0; -0.1];

dt = 0.01;
totalT = norm(A-B)/norm(vAB);
t = 0:dt:totalT;
c = 1;
qp = zeros(size(t,2),3);
dj = zeros(size(t,2),1);
figure()
for i = t
    pos = A + i*vAB;
    p = [pos; phi];
    q(c,:) = inverso3(p,-1);

    ve = [vAB;0];
    qp(c,:) = jacobiano3(q(c,:))\ve;
    dj(c,:) = det(jacobiano3(q(c,:)));
    c = c + 1;

    [T01, T02, T03] = directo3(q(c-1,:));
    x0=0;
    y0=0;
    x1=T01(1,4);

    y1=T01(2,4);
    x2=T02(1,4);
    y2=T02(2,4);
    x3=T03(1,4);
    y3=T03(2,4);
    plot([x0 x1 x2 x3],[y0 y1 y2 y3]);
    ylim([0 0.25])
    xlim([0 0.35])
    hold on
    hold off
    pause(0.01)
end
title('Movement of the manipulator', 'interpreter', ...
'latex','fontsize',18)

figure()
plot(t,q)
title('Evolution of the joint positions', 'interpreter', ...
'latex','fontsize',18)
legend('Joint 1','Joint 2','Joint 3', 'interpreter', ...
'latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
grid

figure()
plot(t,qp)
title('Evolution of the joint velocities', 'interpreter', ...
'latex','fontsize',18)
legend('Joint 1','Joint 2','Joint 3', 'interpreter', ...
'latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
ylabel('$\omega (rad/s)$', 'interpreter', 'latex','fontsize',18)
grid

figure()
plot(t,dj(:,1))
title('Evolution of the jacobian determinant', 'interpreter', ...
'latex','fontsize',18)
grid
