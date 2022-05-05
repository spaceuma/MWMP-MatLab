%% Initialization

addpath(genpath('../../src'))
addpath('../../simscape')

clear

tic
% System properties
global d0;
d0 = 0.50;
global a1;
a1 = 0.225;
global a2;
a2 = 0.735;
global d4;
d4 = 0.695;

global d6; %Dummy distance for gripper visualization
d6 = 0.14;

global armHeight;
armHeight = 0.05;
global armWidth;
armWidth = 0.1;

global rho;
rho = 2700;

global m1 m2 m3;
m1 = 3;
m2 = 9;
m3 = 9;

global g;
g = 0;

%% Constraints 
% Initial configuration
qi = [0, -pi/2, pi/2];

% Initial end effector pose
[~, ~, ~, TW3] = direct3(qi);

xei = TW3(1,4);
yei = TW3(2,4);
zei = TW3(3,4);
rollei = 0;
pitchei = pi/2;
yawei = 0;

% Goal end effector pose
xef = -0.5;
yef = 0.3;
zef = 1.5;
rollef = 0;
pitchef = pi;
yawef = 0;

%% Time horizon
expectedTimeArrival = 5;
tf = expectedTimeArrival; % Time vector
dt = tf/199;
t = 0:dt:tf;

%% Costs
% State costs
fc = 10000000000000000; % Final state cost, 1000000000
foc = 0; % Final orientation cost, 0
fsc = 0; % Final zero speed cost, 1000000000

% tau1c = 0.005; % Joint 1 inverse torque constant, 2
% tau2c = 0.005; % Joint 2 inverse torque constant, 2
% tau3c = 0.005; % Joint 3 inverse torque constant, 2

% Input costs
ac1 = 1000000000; % Arm actuation cost
ac2 = 1000000000; % Arm actuation cost
ac3 = 1000000000; % Arm actuation cost

%% Other variables
% Minimum step actuation percentage
lineSearchStep = 0.30; 

% Max acceptable dist
distThreshold = 0.005;

% Maximum number of iterations
maxIter = 150;

%% State space model
% State vectors
numStates = 12;
x = zeros(numStates,size(t,2));
% WTEE
x(1,1) = xei;
x(2,1) = yei;
x(3,1) = zei;
x(4,1) = rollei;
x(5,1) = pitchei;
x(6,1) = yawei;
% Arm joints positions
x(7,1) = qi(1);
x(8,1) = qi(2);
x(9,1) = qi(3);
% Arm joints speeds
x(10,1) = 0;
x(11,1) = 0;
x(12,1) = 0;

% Initial control law
numInputs = 3;
u = zeros(numInputs,size(t,2));

% Target state and control trajectories
x0 = zeros(numStates,size(t,2));

% WTEE
x0(1,end) = xef;
x0(2,end) = yef;
x0(3,end) = zef;
x0(4,end) = rollef;
x0(5,end) = pitchef;
x0(6,end) = yawef;
% Arm joints positions
x0(7,end) = 0;
x0(8,end) = 0;
x0(9,end) = 0;
% Arm joints speeds
x0(10,end) = 0;
x0(11,end) = 0;
x0(12,end) = 0;

u0 = zeros(numInputs,size(t,2));

Jac = zeros(6,3,size(t,2));


%% SLQR algorithm
iter = 1;
while 1   
    % Forward integrate system equations
    x = forwardIntegrateSystem(x, u, dt, g);
    
    % Checking the state
    
    figure(1)
    hold off;
    % Plotting first arm config
    [TW0, TW1, TW2, TW3] = direct3(x(7:9,1));
    plot3([0 TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
          [0 TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
          [0 TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)], 'Color', 'r', 'LineWidth', 2.5);
    hold on;

    % Plotting last arm config
    [TW0, TW1, TW2, TW3] = direct3(x(7:9,end));
    plot3([0 TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
          [0 TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
          [0 TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)], 'Color', 'r', 'LineWidth', 2.5);
    hold on;

    % Plotting scenario
    daspect([1 1 1])
    plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 5, 'Color', 'y')
    title('Manipulator trajectories', 'interpreter', ...
    'latex','fontsize',18)
    plot3(xei,yei,zei, 'MarkerSize', 20, 'Marker', '.', 'Color', 'b')
    plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', 'c')
    
    quiver3(0, 0, 0, 1/4, 0, 0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.7)
    quiver3(0, 0, 0, 0, 1/4, 0, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.7)
    quiver3(0, 0, 0, 0, 0, 1/4, 'Color', 'c', 'LineWidth', 2, 'MaxHeadSize', 0.7) 

    hold off;   
    
    
    % Update reference trajectories    
    xh0 = x0 - x;
    uh0 = u0 - u;    
    
    % Quadratize cost function along the trajectory
    Q = zeros(numStates,numStates,size(t,2));
    
    Q(:,:,end) = zeros(numStates,numStates);
    
    Q(1,1,end) = fc;
    Q(2,2,end) = fc;
    Q(3,3,end) = fc;
    Q(4,4,end) = foc;
    Q(5,5,end) = foc;
    Q(6,6,end) = foc;
    Q(10,10,end) = fsc;
    Q(11,11,end) = fsc;
    Q(12,12,end) = fsc;
    
    R = eye(numInputs);
    R(1,1) = ac1;
    R(2,2) = ac2;
    R(3,3) = ac3;
    
    % Linearize the system dynamics and constraints along the trajectory  
    for i = 2:size(t,2)
        Jac(:,:,i-1) = jacobian3(x(7:9,i-1));
    end
    
    % State (x) matrix
    A = zeros(numStates,numStates,size(t,2));

    % W2EE
    A(1:6,1:6,1) = eye(6,6);
    A(1:6,10:12,1) = Jac(:,:,1)*dt;
    
    % Arm joints Position
    A(7:9,7:9,1) = eye(3,3);
    A(7:9,10:12,1) = dt*eye(3,3);
        
    % Arm joints velocities
    A(10:12,10:12,1) = eye(3,3) - dt*inv(getB3(x(7,1), x(8,1), x(9,1)))*...
                                     getC3(x(7,1), x(8,1), x(9,1),...
                                           x(10,1), x(11,1), x(12,1));
                
    for i = 2:size(t,2)
        % W2EE
        A(1:6,1:6,i) = eye(6,6);
        A(1:6,10:12,i) = Jac(:,:,i-1)*dt;

        % Arm joints Position
        A(7:9,7:9,i) = eye(3,3);
        A(7:9,10:12,i) = dt*eye(3,3);

        % Arm joints velocities
        A(10:12,10:12,i) = eye(3,3) - dt*inv(getB3(x(7,i-1), x(8,i-1), x(9,i-1)))*...
                                         getC3(x(7,i-1), x(8,i-1), x(9,i-1),...
                                               x(10,i-1), x(11,i-1), x(12,i-1));
    end
    
    % Actuation (u) matrix
    B = zeros(numStates,numInputs,size(t,2));
        
    % Arm joints acceleration
    B(10:12,1:3,1) = dt*inv(getB3(x(7,1), x(8,1), x(9,1)));
    
    for i = 2:size(t,2)
        % Arm joints acceleration
        B(10:12,1:3,i) = dt*inv(getB3(x(7,i-1), x(8,i-1), x(9,i-1)));
    end
            
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
    
    P(:,:,end) = Q(:,:,end);
    s(:,:,end) = -Q(:,:,end)*xh0(:,end);
    
    xh = zeros(size(x,1),size(t,2));
    uh = zeros(size(u,1),size(t,2));
    v = zeros(size(xh));
    lambdah = zeros(size(s));
        
    % Solve backward
    for i = size(t,2)-1:-1:1
        M(:,:,i) = inv(eye(numStates) + B(:,:,i)/R*B(:,:,i).'*P(:,:,i+1));
        P(:,:,i) = Q(:,:,i) + A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*A(:,:,i);
        s(:,:,i) = A(:,:,i).'*(eye(size(Q,1)) - ...
                   P(:,:,i+1)*M(:,:,i)*B(:,:,i)/R*B(:,:,i).')*s(:,:,i+1)+...
                   A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*B(:,:,i)*uh0(:,i) -...
                   Q(:,:,i)*xh0(:,i);
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
    endDist = norm(x(1:3,end)-x0(1:3,end));
    disp(['Iteration number ',num2str(iter-1),...
          ', distance to goal = ',num2str(endDist)]);
    
    if (norm(uh)<=0.00000001*norm(u) ||...
       (norm(uh)<=0.2*norm(u))&&(endDist<distThreshold))
        disp(['SLQ found the optimal control input within ',...
              num2str(iter-1),' iterations'])
        break;
    else
        % Line search to optimize alfa
        alfa = 1:-lineSearchStep:0.0001;
        J = zeros(size(alfa));
        uk = u;
        for n = 1:size(alfa,2)
            u = uk + alfa(n)*uh;
            x = forwardIntegrateSystem(x, u, dt, g);

            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Q(:,:,end)*(x(:,end)-x0(:,end));
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
        cprintf('err','MMKP failed to generate a motion plan\n')
        if norm(uh)>0.0001*norm(u)
            error('The SLQR algorithm failed to converge');
        elseif endDist > distThreshold
            error('The goal was not reachable');        
        else
            error('Something unexpected prevented SLQR to converge');
        end
    end
              
end

%% Plots
    
x = forwardIntegrateSystem(x, u, dt, g);

toc
iu = cumsum(abs(u(1,:))*dt);
disp(['Total torque applied arm joint 1: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(u(2,:))*dt);
disp(['Total torque applied arm joint 2: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(u(3,:))*dt);
disp(['Total torque applied arm joint 3: ',num2str(iu(end)),' Nm'])    

figure(1)
hold off;
% Plotting first arm config
[TW0, TW1, TW2, TW3] = direct3(x(7:9,1));
plot3([0 TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
      [0 TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
      [0 TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)], 'Color', 'r', 'LineWidth', 2.5);
hold on;

% Plotting last arm config
[TW0, TW1, TW2, TW3] = direct3(x(7:9,end));
plot3([0 TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
      [0 TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
      [0 TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)], 'Color', 'r', 'LineWidth', 2.5);
hold on;

% Plotting scenario
daspect([1 1 1])
plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 5, 'Color', 'y')
title('Manipulator trajectories', 'interpreter', ...
'latex','fontsize',18)
plot3(xei,yei,zei, 'MarkerSize', 20, 'Marker', '.', 'Color', 'b')
plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', 'c')

quiver3(0, 0, 0, 1/4, 0, 0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.7)
quiver3(0, 0, 0, 0, 1/4, 0, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.7)
quiver3(0, 0, 0, 0, 0, 1/4, 'Color', 'c', 'LineWidth', 2, 'MaxHeadSize', 0.7) 

hold off;


% figure(2)
% plot(t,x(7:9,:))
% title('Evolution of the arm joints position', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\theta_1$','$\theta_2$','$\theta_3$', 'interpreter', ...
%        'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
% figure(3)
% plot(t,x(10:12,:))
% title('Evolution of the arm joints speed','interpreter','latex')
% xlabel('t(s)','interpreter','latex','fontsize',18)
% ylabel('$\dot\theta(rad/s$)','interpreter','latex','fontsize',18)
% legend('$\dot\theta_1$','$\dot\theta_2$',...
%        '$\dot\theta_3$','interpreter', ...
%        'latex','fontsize',18)

% figure(4)
% plot(t,x(13:15,:))
% title('Evolution of the arm joints accelerations', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\ddot\theta_1$','$\ddot\theta_2$','$\ddot\theta_3$', 'interpreter', ...
%        'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\ddot\theta (rad/s^2)$', 'interpreter', 'latex','fontsize',18)
% grid

% figure(5)
% plot(t,u(1:3,:))
% title('Evolution of the applied arm torques', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\tau_1$','$\tau_2$','$\tau_3$', 'interpreter', ...
%        'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\tau (Nm)$', 'interpreter', 'latex','fontsize',18)
% grid


%% Simulation
% sim('manipulator3DoF_torques',t(end));
% sim('manipulator3DoF',t(end));


