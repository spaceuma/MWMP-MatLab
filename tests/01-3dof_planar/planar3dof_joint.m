init = cputime;
% System properties
a1 = 0.20;
a2 = 0.15;
a3 = 0.10;

% Constraints 
ti = 0;
xei = 0.2;
yei = 0;
thetai = 0;

tf = 15;
xef = 0.1;
yef = 0.2;
thetaf = 0;
 
dt = 0.01;
t = 0:dt:tf;

% State vector
x = zeros(6,size(t,2));
qi = inverso3([xei yei thetai],-1);
x(1,1) = qi(1);
x(2,1) = 0;
x(3,1) = qi(2);
x(4,1) = 0;
x(5,1) = qi(3);
x(6,1) = 0;

% Initial control law
u = zeros(3,size(t,2));

% Target state and control trajectories
x0 = zeros(6,size(t,2));
qf = inverso3([xef yef thetaf],-1);
x0(1,end) = qf(1);
x0(2,end) = 0;
x0(3,end) = qf(2);
x0(4,end) = 0;
x0(5,end) = qf(3);
x0(6,end) = 0;

u0 = zeros(3,size(t,2));

% SLQR algorithm
iter = 1;
while 1   
    % Forward integrate system equations    
    for i = 2:size(t,2)        
        x(1,i) = x(1,i-1) + x(2,i-1)*dt;
        x(2,i) = x(2,i-1) + u(1,i-1)*dt;
        x(3,i) = x(3,i-1) + x(4,i-1)*dt;
        x(4,i) = x(4,i-1) + u(2,i-1)*dt;
        x(5,i) = x(5,i-1) + x(6,i-1)*dt;
        x(6,i) = x(6,i-1) + u(3,i-1)*dt;
    end
    
    xh0 = x0 - x;
    uh0 = u0 - u;
    
    % Quadratize cost function along the trajectory
    Q = [0 0 0 0 0 0;
         0 0 0 0 0 0;
         0 0 0 0 0 0;
         0 0 0 0 0 0;
         0 0 0 0 0 0;
         0 0 0 0 0 0];
     
    Qend = [1000 0 0 0 0 0;
            0 1000 0 0 0 0;
            0 0 1000 0 0 0;
            0 0 0 1000 0 0;
            0 0 0 0 1000 0;
            0 0 0 0 0 1000];
        
    R = [1 0 0;
         0 1 0;
         0 0 1];

    % Linearize the system dynamics and constraints along the trajectory    
    A = [1 dt 0 0  0 0;
         0 1  0 0  0 0;
         0 0  1 dt 0 0;
         0 0  0 1  0 0;
         0 0  0 0  1 dt;
         0 0  0 0  0 1];
     
    B = [0  0  0;
         dt 0  0;
         0  0  0;
         0  dt 0;
         0  0  0;
         0  0  dt];
    
    
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
    P(:,:,end) = Qend;
    s(:,:,end) = -Qend*xh0(:,end);
    
    xh = zeros(6,size(t,2));
    uh = zeros(3,size(t,2));
    v = zeros(size(xh));
    lambdah = zeros(size(s));    
%     xh(1,1) = qi(1);
%     xh(2,1) = 0;
%     xh(3,1) = qi(2);
%     xh(4,1) = 0;
%     xh(5,1) = qi(3);
%     xh(6,1) = 0;
    
    % Solve backward
    for i = size(t,2)-1:-1:1
        M(:,:,i) = inv(eye(size(B,1)) + B*inv(R)*B.'*P(:,:,i+1));
        P(:,:,i) = Q + A.'*P(:,:,i+1)*M(:,:,i)*A;
        s(:,:,i) = A.'*(eye(size(Q,1)) - P(:,:,i+1)*M(:,:,i)*B*inv(R)*B.')*s(:,:,i+1)+...
            A.'*P(:,:,i+1)*M(:,:,i)*B*uh0(:,i) - Q*xh0(:,i);
    end
    
    % Solve forward
    for i = 1:size(t,2)-1
        v(:,i) = M(:,:,i)*B*(uh0(:,i)-inv(R)*B.'*s(:,:,i+1));
        xh(:,i+1) = M(:,:,i)*A*xh(:,i)+v(:,i);
        lambdah(:,:,i+1) = P(:,:,i+1)*xh(:,i+1)+s(:,:,i+1);
        uh(:,i) = uh0(:,i)-inv(R)*B.'*lambdah(:,:,i+1);
    end
    
    % Exit condition
    if norm(uh)<0.000001*norm(u)
        break;
    else
        % Line search to optimize alfa
        alfa = 1:-0.1:0.0001;
        J = zeros(size(alfa));
        uk = u;
        for n = 1:size(alfa,2)
            u = uk + alfa(n)*uh;
            for i = 2:size(t,2)
                x(1,i) = x(1,i-1) + x(2,i-1)*dt;
                x(2,i) = x(2,i-1) + u(1,i-1)*dt;
                x(3,i) = x(3,i-1) + x(4,i-1)*dt;
                x(4,i) = x(4,i-1) + u(2,i-1)*dt;
                x(5,i) = x(5,i-1) + x(6,i-1)*dt;
                x(6,i) = x(6,i-1) + u(3,i-1)*dt;
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
    end
    
%     disp(iter)
    iter = iter+1;
end
endt = cputime;
disp(['SLQ found the optimal control input within ',num2str(iter-1),' iterations'])
disp(['Elapsed execution time: ',num2str(endt-init),' seconds'])
iu = cumsum(abs(u(1,:)));
disp(['Total acc applied joint 1: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(2,:)));
disp(['Total acc applied joint 2: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(3,:)));
disp(['Total acc applied joint 3: ',num2str(iu(end)),' m/s^2'])

figure()
plot(t,x)
title('Evolution of the joint positions and velocities', 'interpreter', ...
'latex','fontsize',18)
legend('Joint 1 pos','Joint 1 vel','Joint 2 pos','Joint 2 vel','Joint 3 pos','Joint 3 vel', 'interpreter', ...
'latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
grid

figure()
[T01, T02, T03] = directo3([x(1,1),x(3,1),x(5,1)]);
x0=0;
y0=0;
x1=T01(1,4);
y1=T01(2,4);
x2=T02(1,4);
y2=T02(2,4);
x3=T03(1,4);
y3=T03(2,4);
plot([x0 x1 x2 x3],[y0 y1 y2 y3]);
hold on;
[T01, T02, T03] = directo3([x(1,end),x(3,end),x(5,end)]);
x0=0;
y0=0;
x1=T01(1,4);
y1=T01(2,4);
x2=T02(1,4);
y2=T02(2,4);
x3=T03(1,4);
y3=T03(2,4);
plot([x0 x1 x2 x3],[y0 y1 y2 y3]);
title('State of the manipulator', 'interpreter', ...
'latex','fontsize',18)

figure()
plot(t,u)
title('Actuating acc (u)','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('$a(m/s^2$)','interpreter','latex')
hold on



