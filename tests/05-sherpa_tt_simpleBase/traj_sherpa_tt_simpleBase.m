init = cputime;
% System properties
global a1;
a1 = 0.20;
global a2;
a2 = 0.15;
global a3;
a3 = 0.10;
global a4;
a4 = 0.05;
global a5;
a5 = 0.04;

% Constraints 
xc0 = 0.0;

ti = 0;
xei = 0.2 + xc0;
yei = -0.09;
thetai = -pi/2;

tf = 15;
xef = 0.5;
yef = -0.09;
thetaf = -pi/2;
 
dt = 0.08;
t = 0:dt:tf;

% State vector
q = zeros(5,size(t,2));
q(:,1) = [0.8128; -2.6362; 1.8235; -pi/2; 0];

xc = zeros(1,size(t,2));
xc(1) = xc0;

x = zeros(9,size(t,2));
x(1,1) = xei;
x(2,1) = yei;
x(3,1) = thetai;
x(4,1) = 0;
x(5,1) = 0;
x(6,1) = 0;
x(7,1) = 0;
x(8,1) = 0;
x(9,1) = 0;

% Initial control law
u = zeros(6,size(t,2));

% Target state and control trajectories
x0 = zeros(8,size(t,2));
x0(1,1) = xei;
x0(2,1) = yei;
x0(3,1) = thetai;
x0(4,1) = 0;
x0(5,1) = 0;
x0(6,1) = 0;
x0(7,1) = 0;
x0(8,1) = 0;
x0(9,1) = 0;
x0(1,end) = xef;
x0(2,end) = yef;
x0(3,end) = thetaf;
x0(4,end) = 0;
x0(5,end) = 0;
x0(6,end) = 0;
x0(7,end) = 0;
x0(8,end) = 0;
x0(9,end) = 0;

for i = 2:size(t,2)
    x0(1:3,i) = (x0(1:3,end)-x0(1:3,1))*i/size(t,2) + x0(1:3,1);    
end

u0 = zeros(6,size(t,2));

Jac = zeros(3,5,size(t,2));

% SLQR algorithm
iter = 1;
while 1   
    % Forward integrate system equations
    for i = 2:size(t,2)
        x(4:9,i) = x(4:9,i-1) + u(:,i-1)*dt; 
        q(:,i) = q(:,i-1) + x(5:9,i-1)*dt;
        xc(i)  = xc(i-1) + x(4,i-1)*dt;
        Jac(:,:,i-1) = jacobiano5(q(:,i-1));
        x(1,i) = x(1,i-1) + Jac(1,:,i-1)*x(5:9,i-1)*dt + x(4,i-1)*dt; 
        x(2:3,i) = x(2:3,i-1) + Jac(2:3,:,i-1)*x(5:9,i-1)*dt;             
    end
    Jac(:,:,end) = jacobiano5(q(:,end));
    
    xh0 = x0 - x;
    uh0 = u0 - u;
    
    
    % Quadratize cost function along the trajectory
    Q = [1000 0 0 0 0 0 0 0 0;
         0 1000 0 0 0 0 0 0 0;
         0 0 1000 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0];
     
    Qend = [1000 0    0    0 0 0 0 0 0;
            0    1000 0    0 0 0 0 0 0;
            0    0    1000 0 0 0 0 0 0;
            0    0    0    0 0 0 0 0 0;
            0    0    0    0 0 0 0 0 0;
            0    0    0    0 0 0 0 0 0;
            0    0    0    0 0 0 0 0 0;
            0    0    0    0 0 0 0 0 0;
            0    0    0    0 0 0 0 0 0];
        
    R = [10000 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 1 0 0 0;
         0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1];

    % Linearize the system dynamics and constraints along the trajectory    
    A = zeros(size(x,1),size(x,1),size(t,2));
    for i = 1:size(t,2)
        A(:,:,i) = [1 0 0 dt dt*Jac(1,1,i) dt*Jac(1,2,i) dt*Jac(1,3,i) dt*Jac(1,4,i) dt*Jac(1,5,i);
                    0 1 0 0  dt*Jac(2,1,i) dt*Jac(2,2,i) dt*Jac(2,3,i) dt*Jac(2,4,i) dt*Jac(2,5,i);
                    0 0 1 0  dt*Jac(3,1,i) dt*Jac(3,2,i) dt*Jac(3,3,i) dt*Jac(3,4,i) dt*Jac(3,5,i);
                    0 0 0 1  0             0             0             0             0;
                    0 0 0 0  1             0             0             0             0;
                    0 0 0 0  0             1             0             0             0;
                    0 0 0 0  0             0             1             0             0;
                    0 0 0 0  0             0             0             1             0;
                    0 0 0 0  0             0             0             0             1];
    end
    
    B = zeros(size(x,1),size(u,1),size(t,2));
    for i = 1:size(t,2)
        B(:,:,i) = [0  0  0  0  0  0;
                    0  0  0  0  0  0;
                    0  0  0  0  0  0;
                    dt 0  0  0  0  0;
                    0  dt 0  0  0  0;
                    0  0  dt 0  0  0;
                    0  0  0  dt 0  0;
                    0  0  0  0  dt 0;
                    0  0  0  0  0  dt];
    end    
    
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
    P(:,:,end) = Qend;
    s(:,:,end) = -Qend*xh0(:,end);
    
    xh = zeros(9,size(t,2));
    uh = zeros(6,size(t,2));
    v = zeros(size(xh));
    lambdah = zeros(size(s));
    
%     xh(1,1) = 0;
%     xh(2,1) = yei;
%     xh(3,1) = thetai;
%     xh(4,1) = 0;
%     xh(5,1) = 0;
%     xh(6,1) = 0;
    
    % Solve backward
    for i = size(t,2)-1:-1:1
        M(:,:,i) = inv(eye(size(B,1)) + B(:,:,i)*inv(R)*B(:,:,i).'*P(:,:,i+1));
        P(:,:,i) = Q + A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*A(:,:,i);
        s(:,:,i) = A(:,:,i).'*(eye(size(Q,1)) - P(:,:,i+1)*M(:,:,i)*B(:,:,i)*inv(R)*B(:,:,i).')*s(:,:,i+1)+...
            A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*B(:,:,i)*uh0(:,i) - Q*xh0(:,i);
    end
    
    % Solve forward
    for i = 1:size(t,2)-1
        v(:,i) = M(:,:,i)*B(:,:,i)*(uh0(:,i)-inv(R)*B(:,:,i).'*s(:,:,i+1));
        xh(:,i+1) = M(:,:,i)*A(:,:,i)*xh(:,i)+v(:,i);
        lambdah(:,:,i+1) = P(:,:,i+1)*xh(:,i+1)+s(:,:,i+1);
        uh(:,i) = uh0(:,i)-inv(R)*B(:,:,i).'*lambdah(:,:,i+1);
    end
    
    % Exit condition
    if norm(uh)<0.001*norm(u)
        break;
    else
        % Line search to optimize alfa
        alfa = 1:-0.1:0.0001;
        J = zeros(size(alfa));
        uk = u;
        for n = 1:size(alfa,2)
            u = uk + alfa(n)*uh;
            for i = 2:size(t,2)
                x(4:9,i) = x(4:9,i-1) + u(:,i-1)*dt; 
                q(:,i) = q(:,i-1) + x(5:9,i-1)*dt;
                xc(i)  = xc(i-1) + x(4,i-1)*dt;
                Jac(:,:,i-1) = jacobiano5(q(:,i-1));
                x(1,i) = x(1,i-1) + Jac(1,:,i-1)*x(5:9,i-1)*dt + x(4,i-1)*dt; 
                x(2:3,i) = x(2:3,i-1) + Jac(2:3,:,i-1)*x(5:9,i-1)*dt; 
            end
            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Qend*(x(:,end)-x0(:,end));
            for i = 1:size(t,2)-1
                J(n) = J(n) + 1/2*((x(:,i)-x0(:,i)).'*Q*(x(:,i)-x0(:,i)) + (u(:,i)-u0(:,i)).'*R*(u(:,i)-u0(:,i)));
            end            
        end
        [mincost, ind] = min(J);
        alfamin = alfa(ind);
        
        % Update controller
        u = uk + alfamin*uh;
%         u = u + uh;

    end
    
    iter = iter+1;
    
    figure(1)
    hold off;
    [Tc0, Tc1, Tc2, Tc3, Tc4, Tc5] = directo5(q(:,1), xc(1));
    plot([Tc0(1,4) Tc1(1,4) Tc2(1,4) Tc3(1,4) Tc4(1,4) Tc5(1,4)],[0 Tc1(2,4) Tc2(2,4) Tc3(2,4) Tc4(2,4) Tc5(2,4)]);
    hold on;
    [Tc0, Tc1, Tc2, Tc3, Tc4, Tc5] = directo5(q(:,end), xc(end));
    plot([Tc0(1,4) Tc1(1,4) Tc2(1,4) Tc3(1,4) Tc4(1,4) Tc5(1,4)],[0 Tc1(2,4) Tc2(2,4) Tc3(2,4) Tc4(2,4) Tc5(2,4)]);
    plot(x(1,:),x(2,:))
    title('State of the manipulator', 'interpreter', ...
    'latex','fontsize',18)   
end

for i = 2:size(t,2)
    x(4:9,i) = x(4:9,i-1) + u(:,i-1)*dt; 
    q(:,i) = q(:,i-1) + x(5:9,i-1)*dt;
    xc(i)  = xc(i-1) + x(4,i-1)*dt;
    Jac(:,:,i-1) = jacobiano5(q(:,i-1));
    x(1,i) = x(1,i-1) + Jac(1,:,i-1)*x(5:9,i-1)*dt + x(4,i-1)*dt; 
    x(2:3,i) = x(2:3,i-1) + Jac(2:3,:,i-1)*x(5:9,i-1)*dt;             
end

endt = cputime;
disp(['SLQ found the optimal control input within ',num2str(iter-1),' iterations'])
disp(['Elapsed execution time: ',num2str(endt-init),' seconds'])
iu = cumsum(abs(u(1,:)));
disp(['Total acc applied x displacement: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(2,:)));
disp(['Total acc applied joint 1: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(3,:)));
disp(['Total acc applied joint 2: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(4,:)));
disp(['Total acc applied joint 3: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(5,:)));
disp(['Total acc applied joint 4: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(6,:)));
disp(['Total acc applied joint 5: ',num2str(iu(end)),' m/s^2'])

% figure(2)
% plot(t,x)
% title('Evolution of the state', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$x_{ee}$','$y_{ee}$','$\theta_{ee}$','$x^._{c}$','$\tau_1^.$','$\tau_2^.$','$\tau^._3$','$\tau_4^.$','$\tau_5^.$', 'interpreter', ...
% 'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
% grid

figure(1)
hold off;
[Tc0, Tc1, Tc2, Tc3, Tc4, Tc5] = directo5(q(:,1), xc(1));
plot([Tc0(1,4) Tc1(1,4) Tc2(1,4) Tc3(1,4) Tc4(1,4) Tc5(1,4)],[0 Tc1(2,4) Tc2(2,4) Tc3(2,4) Tc4(2,4) Tc5(2,4)]);
hold on;
[Tc0, Tc1, Tc2, Tc3, Tc4, Tc5] = directo5(q(:,end), xc(end));
plot([Tc0(1,4) Tc1(1,4) Tc2(1,4) Tc3(1,4) Tc4(1,4) Tc5(1,4)],[0 Tc1(2,4) Tc2(2,4) Tc3(2,4) Tc4(2,4) Tc5(2,4)]);
plot(x(1,:),x(2,:))
title('State of the manipulator', 'interpreter', ...
'latex','fontsize',18) 

% figure(3)
% plot(t,u)
% title('Actuating acc (u)','interpreter','latex')
% xlabel('t(s)','interpreter','latex')
% ylabel('$a(m/s^2$)','interpreter','latex')
% legend('x displacement','$Joint 1$','$Joint 2$','$Joint 3$','$Joint 4$','$Joint 5$', 'interpreter', ...
% 'latex','fontsize',18)
% hold on

sim('base_dof5Manip_planar',15);


