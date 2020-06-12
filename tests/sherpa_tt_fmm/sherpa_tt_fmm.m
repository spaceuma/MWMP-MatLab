addpath('../../../ARES-DyMu_matlab/Global Path Planning/functions')
addpath('../../maps')
addpath('../../models')
addpath('../../costs')

init = cputime;
% System properties
global d0;
d0 = 0.50;
global a1;
a1 = 0.225;
global a2;
a2 = 0.735;
global c2;
c2 = 0.030;
global a3;
a3 = 0.030;
global d4;
d4 = 0.695;
global d6;
d6 = 0.30;

safetyDistance = 1;
mapResolution = 0.05;
vehicleSpeed = 0.1;

% Constraints 
xB0 = 3;
yB0 = 3;
zB0 = 0.645;
yawB0 = 0;

TWB = getTraslation([xB0,yB0,zB0])*getZRot(yawB0);
[~, ~, ~, ~, ~, ~, TB6] = direct([0, -pi/2, pi/2, 0, pi/2, 0]);

TW6 = TWB*TB6;

xei = TW6(1,4);
yei = TW6(2,4);
zei = TW6(3,4);
rollei = 0;
pitchei = pi;
yawei = 0;

xef = 6;
yef = 7;
zef = 1;
rollef = 0;
pitchef = pi;
yawef = 0;

% FMM to compute reference path
load('obstMap2','obstMap')
dilatedObstMap = dilateObstMap(obstMap, safetyDistance, mapResolution);
distMap = mapResolution*bwdist(dilatedObstMap);
costMap = 1./distMap;
costMap = costMap./min(min(costMap));

iInit = [round(xB0/mapResolution)+1 round(yB0/mapResolution)+1];
iGoal = [round(xef/mapResolution)+1 round(yef/mapResolution)+1];

[totalCostMap, ~] = computeTmap(costMap,iGoal);

tau = 0.5;
[referencePath,~,~,~] = getPathGDM(totalCostMap,iInit,iGoal,tau);
referencePath = (referencePath-1)*mapResolution;

% Compute time needed to obtain control horizon
distanceCovered = 0;
for i = 2:size(referencePath,1)
    distanceCovered = distanceCovered+sqrt((referencePath(i,1)-referencePath(i-1,1))^2+(referencePath(i,2)-referencePath(i-1,2))^2);
end

tf = distanceCovered/vehicleSpeed;
dt = tf/(size(referencePath,1)-1);


% State vectors
t = 0:dt:tf;

q = zeros(6,size(t,2));
q(:,1) = [0, -pi/2, pi/2, 0, pi/2, 0];

x = zeros(16,size(t,2));
x(1,1) = xei;
x(2,1) = yei;
x(3,1) = zei;
x(4,1) = rollei;
x(5,1) = pitchei;
x(6,1) = yawei;
x(7,1) = 0;
x(8,1) = 0;
x(9,1) = 0;
x(10,1) = 0;
x(11,1) = 0;
x(12,1) = 0;
x(13,1) = 0;
x(14,1) = 0;
x(15,1) = xB0;
x(16,1) = yB0;

% Initial control law
u = zeros(8,size(t,2));

% Target state and control trajectories
x0 = zeros(16,size(t,2));
x0(1,end) = xef;
x0(2,end) = yef;
x0(3,end) = zef;
x0(4,end) = rollef;
x0(5,end) = pitchef;
x0(6,end) = yawef;
x0(7,end) = 0;
x0(8,end) = 0;
x0(9,end) = 0;
x0(10,end) = 0;
x0(11,end) = 0;
x0(12,end) = 0;
x0(13,end) = 0;
x0(14,end) = 0;
x0(15,:) = referencePath(:,1)';
x0(16,:) = referencePath(:,2)';

u0 = zeros(8,size(t,2));

Jac = zeros(6,6,size(t,2));

% SLQR algorithm
iter = 1;
while 1   
    % Forward integrate system equations
    for i = 2:size(t,2)
        q(:,i) = q(:,i-1) + x(9:14,i-1)*dt;
        Jac(:,:,i-1) = jacobian(q(:,i-1));
        x(1:2,i) = x(1:2,i-1) + Jac(1:2,:,i-1)*x(9:14,i-1)*dt + x(7:8,i-1)*dt; 
        x(3:6,i) = x(3:6,i-1) + Jac(3:6,:,i-1)*x(9:14,i-1)*dt;    
        x(7:14,i) = x(7:14,i-1) + u(:,i-1)*dt; 
        x(15:16,i) = x(15:16,i-1) + x(7:8,i-1)*dt;
    end
    Jac(:,:,end) = jacobian(q(:,end));
    
    xh0 = x0 - x;
    uh0 = u0 - u;    
    
    % Quadratize cost function along the trajectory
    Q = zeros(size(x,1),size(x,1),size(t,2));
    Q(15,15,1:end-round(size(t,2)/10)) = 0.02;
    Q(16,16,1:end-round(size(t,2)/10)) = 0.02;
     
    Qend = zeros(size(x,1),size(x,1));
    Qend(1,1) = 100000000;
    Qend(2,2) = 100000000;
    Qend(3,3) = 100000000;
    Qend(4,4) = 100000000;
    Qend(5,5) = 100000000;
    Qend(6,6) = 100000000;
    
    R = 1*eye(size(u,1));
    R(1,1) = 1;
    R(2,2) = 1;
    
    % Linearize the system dynamics and constraints along the trajectory    
    A = zeros(size(x,1),size(x,1),size(t,2));
    for i = 1:size(t,2)
        A(:,:,i) = eye(size(x,1),size(x,1));
        A(1,7,i) = dt;
        A(2,8,i) = dt;
        A(1:6,9:14,i) = Jac(:,:,i);
        A(15,15,i) = 1;
        A(15,7,i) = dt;
        A(16,16,i) = 1;
        A(16,8,i) = dt;
    end
    
    B = zeros(size(x,1),size(u,1),size(t,2));
    for i = 1:size(t,2)
        B(:,:,i) = [0  0  0  0  0  0  0  0;
                    0  0  0  0  0  0  0  0;
                    0  0  0  0  0  0  0  0;
                    0  0  0  0  0  0  0  0;
                    0  0  0  0  0  0  0  0;
                    0  0  0  0  0  0  0  0;
                    dt 0  0  0  0  0  0  0;
                    0  dt 0  0  0  0  0  0;
                    0  0  dt 0  0  0  0  0;
                    0  0  0  dt 0  0  0  0;
                    0  0  0  0  dt 0  0  0;
                    0  0  0  0  0  dt 0  0;
                    0  0  0  0  0  0  dt 0;
                    0  0  0  0  0  0  0  dt;
                    0  0  0  0  0  0  0  0;
                    0  0  0  0  0  0  0  0];
    end    
    
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
        
    P(:,:,end) = Qend;
    s(:,:,end) = -Qend*xh0(:,end);
    
    xh = zeros(size(x,1),size(t,2));
    uh = zeros(size(u,1),size(t,2));
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
        P(:,:,i) = Q(:,:,i) + A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*A(:,:,i);
        s(:,:,i) = A(:,:,i).'*(eye(size(Q,1)) - P(:,:,i+1)*M(:,:,i)*B(:,:,i)*inv(R)*B(:,:,i).')*s(:,:,i+1)+...
            A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*B(:,:,i)*uh0(:,i) - Q(:,:,i)*xh0(:,i);
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
                q(:,i) = q(:,i-1) + x(9:14,i-1)*dt;
                Jac(:,:,i-1) = jacobian(q(:,i-1));
                x(1:2,i) = x(1:2,i-1) + Jac(1:2,:,i-1)*x(9:14,i-1)*dt + x(7:8,i-1)*dt; 
                x(3:6,i) = x(3:6,i-1) + Jac(3:6,:,i-1)*x(9:14,i-1)*dt;    
                x(7:14,i) = x(7:14,i-1) + u(:,i-1)*dt; 
                x(15:16,i) = x(15:16,i-1) + x(7:8,i-1)*dt;
            end
            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Qend*(x(:,end)-x0(:,end));
            for i = 1:size(t,2)-1
                J(n) = J(n) + 1/2*((x(:,i)-x0(:,i)).'*Q(:,:,i)*(x(:,i)-x0(:,i)) + (u(:,i)-u0(:,i)).'*R*(u(:,i)-u0(:,i)));
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
    [TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(q(:,1));
    plot3([TB0(1,4) TB1(1,4) TB2(1,4) TB3(1,4) TB4(1,4) TB5(1,4) TB6(1,4)]+x(15,1),...
          [TB0(2,4) TB1(2,4) TB2(2,4) TB3(2,4) TB4(2,4) TB5(2,4) TB6(2,4)]+x(16,1),...
          [TB0(3,4) TB1(3,4) TB2(3,4) TB3(3,4) TB4(3,4) TB5(3,4) TB6(3,4)]+zB0);
    hold on;
    [TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(q(:,end));
    plot3([TB0(1,4) TB1(1,4) TB2(1,4) TB3(1,4) TB4(1,4) TB5(1,4) TB6(1,4)]+x(15,end),...
          [TB0(2,4) TB1(2,4) TB2(2,4) TB3(2,4) TB4(2,4) TB5(2,4) TB6(2,4)]+x(16,end),...
          [TB0(3,4) TB1(3,4) TB2(3,4) TB3(3,4) TB4(3,4) TB5(3,4) TB6(3,4)]+zB0);
    plot3(x(1,:),x(2,:),x(3,:))
    plot3(x(15,:),x(16,:),zB0*ones(1,size(t,2)))
    title('State of the manipulator', 'interpreter', ...
    'latex','fontsize',18)   
end

for i = 2:size(t,2)
    q(:,i) = q(:,i-1) + x(9:14,i-1)*dt;
    Jac(:,:,i-1) = jacobian(q(:,i-1));
    x(1:2,i) = x(1:2,i-1) + Jac(1:2,:,i-1)*x(9:14,i-1)*dt + x(7:8,i-1)*dt; 
    x(3:6,i) = x(3:6,i-1) + Jac(3:6,:,i-1)*x(9:14,i-1)*dt;    
    x(7:14,i) = x(7:14,i-1) + u(:,i-1)*dt; 
    x(15:16,i) = x(15:16,i-1) + x(7:8,i-1)*dt;
end

endt = cputime;
disp(['SLQ found the optimal control input within ',num2str(iter-1),' iterations'])
disp(['Elapsed execution time: ',num2str(endt-init),' seconds'])
iu = cumsum(abs(u(1,:)));
disp(['Total acc applied x displacement: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(2,:)));
disp(['Total acc applied y displacement: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(3,:)));
disp(['Total acc applied joint 1: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(4,:)));
disp(['Total acc applied joint 2: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(5,:)));
disp(['Total acc applied joint 3: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(6,:)));
disp(['Total acc applied joint 4: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(7,:)));
disp(['Total acc applied joint 5: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(8,:)));
disp(['Total acc applied joint 6: ',num2str(iu(end)),' m/s^2'])

figure(2)
plot(t,x(7:14,:))
title('Evolution of the state', 'interpreter', ...
'latex','fontsize',18)
legend('$x_{B}^.$','$y_{B}^.$','$\tau_1^.$','$\tau_2^.$','$\tau_3^.$','$\tau_4^.$','$\tau_5^.$','$\tau_6^.$', 'interpreter', ...
'latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
%ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
grid

figure(1)
    hold off;
    [TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(q(:,1));
    plot3([TB0(1,4) TB1(1,4) TB2(1,4) TB3(1,4) TB4(1,4) TB5(1,4) TB6(1,4)]+x(15,1),...
          [TB0(2,4) TB1(2,4) TB2(2,4) TB3(2,4) TB4(2,4) TB5(2,4) TB6(2,4)]+x(16,1),...
          [TB0(3,4) TB1(3,4) TB2(3,4) TB3(3,4) TB4(3,4) TB5(3,4) TB6(3,4)]+zB0);
    hold on;
    [TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(q(:,end));
    plot3([TB0(1,4) TB1(1,4) TB2(1,4) TB3(1,4) TB4(1,4) TB5(1,4) TB6(1,4)]+x(15,end),...
          [TB0(2,4) TB1(2,4) TB2(2,4) TB3(2,4) TB4(2,4) TB5(2,4) TB6(2,4)]+x(16,end),...
          [TB0(3,4) TB1(3,4) TB2(3,4) TB3(3,4) TB4(3,4) TB5(3,4) TB6(3,4)]+zB0);
    plot3(x(1,:),x(2,:),x(3,:))
    plot3(x(15,:),x(16,:),zB0*ones(1,size(t,2)))
    title('State of the manipulator', 'interpreter', ...
    'latex','fontsize',18)  

figure(3)
plot(t,u)
title('Actuating acc (u)','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('$a(m/s^2$)','interpreter','latex')
legend('x displacement','y displacement','$Joint 1$','$Joint 2$',...
       '$Joint 3$','$Joint 4$','$Joint 5$','$Joint 6$', 'interpreter', ...
       'latex','fontsize',18)
hold on

% sim('simulink_sherpa_tt_simpleBase',15);


