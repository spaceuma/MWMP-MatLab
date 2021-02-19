%% Preparing workspace
addpath('../../models')
addpath('../../models/3DoF')

addpath('../../costs')
addpath('../../utils')

clear

tic

%% Initialization
% System properties
% Arm links length
global d0;
d0 = 0.50;
global a1;
a1 = 0.225;
global a2;
a2 = 0.735;
global d4;
d4 = 0.695;

% Cost map
global costMap;
load('costMap','costMap');

% Extra stuff
global armHeight;
armHeight = 0.05;
global armWidth;
armWidth = 0.1;

global dfx;
dfx = 0.7;
global dfy;
dfy = 0.7;
global zBC;
zBC = 0.645;

global reachabilityDistance;
reachabilityDistance = (a1+a2+d4-zBC-d0);

global rho;
rho = 2700;

global r;
r = 0.2;
global wheelWidth;
wheelWidth = 0.2;
global wheelMass;
wheelMass = 5;

global vehicleMass;
vehicleMass = 100;

global m1 m2 m3;
m1 = 3;
m2 = 9;
m3 = 9;

global rollingResistance;
rollingResistance = 0.0036;

global g;
g = 9.81;

% Constraints
% Initial conditions
ti = 0;
qi = [0; -pi/2; pi/2];
rollei = 0;
pitchei = pi/2;
yawei = 0;

xB0 = 3;
yB0 = 3;
zB0 = 0.645;
yawB0 = 0;

TWB = getTraslation([xB0,yB0,zB0])*getZRot(yawB0);
[~, ~, ~, TB3] = direct3(qi);

TW3 = TWB*TB3;

xei = TW3(1,4);
yei = TW3(2,4);
zei = TW3(3,4);

% Goal
tf = 15;
xef = 8;
yef = 5;
zef = 1;
rollef = 0;
pitchef = pi;
yawef = 0;

% Time profile
dt = 0.2;
t = 0:dt:tf;

% State vector
numStates = 11;
x = zeros(numStates,size(t,2));

q = zeros(3,size(t,2));
q(:,1) = qi;

% End effector position
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

xB = zeros(1,size(t,2));
yB = zeros(1,size(t,2));
xB(1) = xB0;
yB(1) = yB0;

% Initial control law
numInputs = 5;
u = zeros(numInputs,size(t,2));

% Target state and control trajectories
x0 = zeros(numStates,size(t,2));
x0(1,end) = xef;
x0(2,end) = yef;
x0(3,end) = zef;
x0(7,end) = 0;
x0(8,end) = 0;
x0(9,end) = 0;
x0(10,end) = 0;
x0(11,end) = 0;

u0 = zeros(numInputs,size(t,2));

Jac = zeros(6,3,size(t,2));

%% SLQR algorithm
lineSearchStep = 0.2;
maxIter = 500; % Maximum number of iterations

iter = 1;
while 1   
    % Forward integrate system equations
    for i = 2:size(t,2)
        % Arm joints position
        q(:,i) = q(:,i-1) + x(9:11,i-1)*dt;
        % Jacobian matrix
        Jac(:,:,i-1) = jacobian3(q(:,i-1));
        % End effector position and orientation
        x(1:2,i) = x(1:2,i-1) + Jac(1:2,:,i-1)*x(9:11,i-1)*dt + x(7:8,i-1)*dt;
        x(3:6,i) = x(3:6,i-1) + Jac(3:6,:,i-1)*x(9:11,i-1)*dt;  
        % Arm joints speed + base speed (x,y)
        x(7:11,i) = x(7:11,i-1) + u(:,i-1)*dt; 
        xB(i) = xB(i-1) + x(7,i)*dt;
        yB(i) = yB(i-1) + x(8,i)*dt;
    end
    Jac(:,:,end) = jacobian3(q(:,end));
    
    % Update reference trajectories
    xh0 = x0 - x;
    uh0 = u0 - u;    
    
    % Quadratize cost function along the trajectory
    Q = zeros(numStates,numStates,size(t,2));
    
    Qend = zeros(numStates,numStates);
    Qend(1,1) = 1000000;
    Qend(2,2) = 1000000;
    Qend(3,3) = 1000000;
    
    R = 1*eye(numInputs);
    R(1,1) = 1;
    R(2,2) = 1;
    
    % Linearize the system dynamics and constraints along the trajectory    
    A = zeros(numStates,numStates,size(t,2));
    for i = 1:size(t,2)
        A(:,:,i) = eye(numStates,numStates);
        A(1,7,i) = dt;
        A(2,8,i) = dt;
        A(1:6,9:11,i) = Jac(:,:,i);
    end
    
    B = zeros(numStates,numInputs,size(t,2));
    for i = 1:size(t,2)
        B(:,:,i) = [0  0  0  0  0;
                    0  0  0  0  0;
                    0  0  0  0  0;
                    0  0  0  0  0;
                    0  0  0  0  0;
                    0  0  0  0  0;
                    dt 0  0  0  0;
                    0  dt 0  0  0;
                    0  0  dt 0  0;
                    0  0  0  dt 0;
                    0  0  0  0  dt];
    end    
    
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
    
    P(:,:,end) = Qend;
    s(:,:,end) = -Qend*xh0(:,end);
    
    xh = zeros(numStates,size(t,2));
    uh = zeros(numInputs,size(t,2));
    v = zeros(size(xh));
    lambdah = zeros(size(s));
       
    % Solve backward
    for i = size(t,2)-1:-1:1
        M(:,:,i) = inv(eye(size(B,1)) + B(:,:,i)/R*B(:,:,i).'*P(:,:,i+1));
        P(:,:,i) = Q(:,:,i) + A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*A(:,:,i);
        s(:,:,i) = A(:,:,i).'*(eye(size(Q,1)) - P(:,:,i+1)*M(:,:,i)*B(:,:,i)/R*B(:,:,i).')*s(:,:,i+1)+...
            A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*B(:,:,i)*uh0(:,i) - Q(:,:,i)*xh0(:,i);
    end
    
    % Solve forward
    for i = 1:size(t,2)-1
        v(:,i) = M(:,:,i)*B(:,:,i)*(uh0(:,i)-R\B(:,:,i).'*s(:,:,i+1));
        xh(:,i+1) = M(:,:,i)*A(:,:,i)*xh(:,i)+v(:,i);
        lambdah(:,:,i+1) = P(:,:,i+1)*xh(:,i+1)+s(:,:,i+1);
        uh(:,i) = uh0(:,i)-R\B(:,:,i).'*lambdah(:,:,i+1);
    end
    
    iter = iter+1;
    
    % Exit condition
    if norm(uh)<0.00001*norm(u)
        disp(['SLQ found the optimal control input within ',num2str(iter-1),' iterations'])
        for i = 2:size(t,2)
            % Arm joints position
            q(:,i) = q(:,i-1) + x(9:11,i-1)*dt;
            % Jacobian matrix
            Jac(:,:,i-1) = jacobian3(q(:,i-1));
            % End effector position and orientation
            x(1:2,i) = x(1:2,i-1) + Jac(1:2,:,i-1)*x(9:11,i-1)*dt + x(7:8,i-1)*dt;
            x(3:6,i) = x(3:6,i-1) + Jac(3:6,:,i-1)*x(9:11,i-1)*dt;  
            % Arm joints speed + base speed (x,y)
            x(7:11,i) = x(7:11,i-1) + u(:,i-1)*dt; 
            xB(i) = xB(i-1) + x(7,i)*dt;
            yB(i) = yB(i-1) + x(8,i)*dt;
        end
        break;
    else
        % Line search to optimize alfa
        alfa = 1:-lineSearchStep:0.0001;
        J = zeros(size(alfa));
        uk = u;
        for n = 1:size(alfa,2)
            u = uk + alfa(n)*uh;
            for i = 2:size(t,2)
                % Arm joints position
                q(:,i) = q(:,i-1) + x(9:11,i-1)*dt;
                % Jacobian matrix
                Jac(:,:,i-1) = jacobian3(q(:,i-1));
                % End effector position and orientation
                x(1:2,i) = x(1:2,i-1) + Jac(1:2,:,i-1)*x(9:11,i-1)*dt + x(7:8,i-1)*dt;
                x(3:6,i) = x(3:6,i-1) + Jac(3:6,:,i-1)*x(9:11,i-1)*dt;  
                % Arm joints speed + base speed (x,y)
                x(7:11,i) = x(7:11,i-1) + u(:,i-1)*dt; 
                xB(i) = xB(i-1) + x(7,i)*dt;
                yB(i) = yB(i-1) + x(8,i)*dt; 
            end
            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Qend*(x(:,end)-x0(:,end));
            for i = 1:size(t,2)-1
                J(n) = J(n) + 1/2*((x(:,i)-x0(:,i)).'*Q(:,:,i)*(x(:,i)-x0(:,i))...
                    + (u(:,i)-u0(:,i)).'*R*(u(:,i)-u0(:,i)));
            end            
        end
        [mincost, ind] = min(J);
        alfamin = alfa(ind);
        
        % Update controller
        u = uk + alfamin*uh;
    end
    
    if iter > maxIter
        cprintf('err','SLQR failed to converge to a solution\n')
        error = 1;
        break;
    end    
    
    figure(1)
    hold off;
    [TB0, TB1, TB2, TB3] = direct3(q(:,1));
    plot3([TB0(1,4) TB1(1,4) TB2(1,4) TB3(1,4)]+xB(1),...
          [TB0(2,4) TB1(2,4) TB2(2,4) TB3(2,4)]+yB(1),...
          [TB0(3,4) TB1(3,4) TB2(3,4) TB3(3,4)]+zB0);
    hold on;
    [TB0, TB1, TB2, TB3] = direct3(q(:,end));
    plot3([TB0(1,4) TB1(1,4) TB2(1,4) TB3(1,4)]+xB(end),...
          [TB0(2,4) TB1(2,4) TB2(2,4) TB3(2,4)]+yB(end),...
          [TB0(3,4) TB1(3,4) TB2(3,4) TB3(3,4)]+zB0);
    plot3(x(1,:),x(2,:),x(3,:))
    plot3(xB(:),yB(:),zB0*ones(1,size(t,2)))
    title('State of the manipulator', 'interpreter', ...
    'latex','fontsize',18)   
end


%% Results
toc

endt = cputime;
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

figure(1)
hold off;
[TB0, TB1, TB2, TB3] = direct3(q(:,1));
plot3([TB0(1,4) TB1(1,4) TB2(1,4) TB3(1,4)]+xB(1),...
      [TB0(2,4) TB1(2,4) TB2(2,4) TB3(2,4)]+yB(1),...
      [TB0(3,4) TB1(3,4) TB2(3,4) TB3(3,4)]+zB0);
hold on;
[TB0, TB1, TB2, TB3] = direct3(q(:,end));
plot3([TB0(1,4) TB1(1,4) TB2(1,4) TB3(1,4)]+xB(end),...
      [TB0(2,4) TB1(2,4) TB2(2,4) TB3(2,4)]+yB(end),...
      [TB0(3,4) TB1(3,4) TB2(3,4) TB3(3,4)]+zB0);
plot3(x(1,:),x(2,:),x(3,:))
plot3(xB(:),yB(:),zB0*ones(1,size(t,2)))
title('State of the manipulator', 'interpreter', ...
'latex','fontsize',18)   

figure(2)
plot(t,x(7:11,:))
title('Evolution of the state', 'interpreter', ...
'latex','fontsize',18)
legend('$x_{B}$','$y_{B}$','$\tau_1^.$','$\tau_2^.$','$\tau^._3$', 'interpreter', ...
'latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
%ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
grid

figure(3)
plot(t,u)
title('Actuating acc (u)','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('$a(m/s^2$)','interpreter','latex')
legend('x displacement','y displacement','$Joint 1$','$Joint 2$',...
       '$Joint 3$', 'interpreter', ...
       'latex','fontsize',18)
hold on

sim('base_3DoF_dynamics_sim.slx',15);


