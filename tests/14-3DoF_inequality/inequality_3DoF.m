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
g = 9.81;

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
xef = 0.5;
yef = -1.0;
zef = 0.0;
rollef = 0;
pitchef = pi;
yawef = 0;


%% Configuration variables
% Number of timesteps
timeSteps = 200;

% Maximum number of iterations
maxIter = 100;

% Activate/deactivate dynamic plotting during the simulation
dynamicPlotting = 0;

% Minimum step actuation percentage
config.lineSearchStep = 0.30; 

% Max acceptable dist
config.distThreshold = 0.005;

% Percentage of constrained timesteps for resampling
config.resamplingThreshold = 30;

% Percentage of step actuation to consider convergence
config.controlThreshold = 1e-3;

%% Time horizon estimation
expectedTimeArrival = 5;
tf = expectedTimeArrival; % Time vector
dt = tf/(timeSteps-1);
t = 0:dt:tf;

%% Costs
time_ratio = tf/5; % Ratio for the costs to ensure convergence

% State costs
fc = 1000000000/time_ratio; % Final state cost, 1000000000
foc = 0/time_ratio; % Final orientation cost, 0
fsc = 1000000000/time_ratio; % Final zero speed cost, 1000000000

tau1c = 0.005/time_ratio; % Joint 1 inverse torque constant, 2
tau2c = 0.005/time_ratio; % Joint 2 inverse torque constant, 2
tau3c = 0.005/time_ratio; % Joint 3 inverse torque constant, 2

% Input costs
ac1 = 100000*time_ratio; % Arm actuation cost
ac2 = 100000*time_ratio; % Arm actuation cost
ac3 = 100000*time_ratio; % Arm actuation cost

%% State space model
% State vectors
numStates = 18;
x = zeros(numStates,timeSteps);
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
% Arm joints accelerations
x(13,1) = 0;
x(14,1) = 0;
x(15,1) = 0;
% Arm joints torques
x(16,1) = 0;
x(17,1) = 0;
x(18,1) = 0;

% Initial control law
numInputs = 3;
u = zeros(numInputs,timeSteps);

% Target state and control trajectories
x0 = zeros(numStates,timeSteps);

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
% Arm joints accelerations
x0(13,end) = 0;
x0(14,end) = 0;
x0(15,end) = 0;
% Arm joints torques
x0(16,end) = 0;
x0(17,end) = 0;
x0(18,end) = 0;

u0 = zeros(numInputs,timeSteps);

% Forward integrate system equations
x = forwardIntegrateSystem(x, u, dt);

%% Constraints matrices definition
% State input constraints
numStateInputConstraints = 0;
I0 = zeros(numStateInputConstraints,timeSteps);
I = I0;
C = zeros(numStateInputConstraints,numStates,timeSteps);
D = zeros(numStateInputConstraints,numInputs,timeSteps);
r = zeros(numStateInputConstraints,timeSteps);

% % The state input constraints are defined as:
% % C*x + D*u + r <= 0
% D(1,3,:) = 1;
% r(1,:) = -0.8;
% 
% D(2,3,:) = -1;
% r(2,:) = -0.8;

% Pure state constraints
numPureStateConstraints = 0;
J0 = zeros(numPureStateConstraints,timeSteps);
J = J0;
G = zeros(numPureStateConstraints,numStates,timeSteps);
h = zeros(numPureStateConstraints,timeSteps);

% % The pure state constraints are defined as:
% % G*x + h <= 0
% G(1,9,:) = 1;
% h(1,:) = -1.8;
% 
% G(2,9,:) = -1;
% h(2,:) = -1.8;

stateSpaceModel.C = C;
stateSpaceModel.D = D;
stateSpaceModel.r = r;
stateSpaceModel.G = G;
stateSpaceModel.h = h;
stateSpaceModel.I = I;
stateSpaceModel.J = J;
stateSpaceModel.I0 = I0;
stateSpaceModel.J0 = J0;

%% Visualization stuff
figure(1)

% Plotting first arm config
[TW0, TW1, TW2, TW3] = direct3(x(7:9,1));
h1 = plot3([0 TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
           [0 TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
           [0 TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)],...
           'Color', [0.8 0.8 0.8], 'LineWidth', 2.5);
hold on;

% Plotting the ee frame
h2 = plotFrame(TW3, 1, 0.1);    


% Plotting last arm config
[TW0, TW1, TW2, TW3] = direct3(x(7:9,end));
h3 = plot3([0 TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
           [0 TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
           [0 TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)],...
           'Color', [0.8 0.8 0.8], 'LineWidth', 2.5);
       
% Plotting the ee frame
h4 = plotFrame(TW3, 1, 0.1);

       
% Plotting starting and goal ee poses
h5 = plot3(xei,yei,zei, 'MarkerSize', 20, 'Marker', '.', 'Color', 'b');
h6 = plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', 'b');

% Plotting the reference frame
h7 = plotFrame([1 0 0 0;
                0 1 0 0;
                0 0 1 0;
                0 0 0 1], 1, 0.2, 'W');
       
% Plotting the ee path
h8 = plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 5, 'Color', 'y');

title('Manipulator trajectories', 'interpreter', ...
      'latex','fontsize',18);
daspect([1 1 1]);
grid
hold off;


%% SLQR algorithm
iter = 1;
while 1      
    % Updating the plot
    if dynamicPlotting
        figure(1);
        hold on;

        delete(h3);
        [TW0, TW1, TW2, TW3] = direct3(x(7:9,end));    
        h3 = plot3([0 TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
                   [0 TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
                   [0 TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)],...
                   'Color', [0.8 0.8 0.8], 'LineWidth', 2.5);
                       
        delete(h4);
        h4 = plotFrame(TW3, 1, 0.1);  
        
        delete(h8);
        h8 = plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 5, 'Color', 'y');  

        hold off;
        
        figure(2)
        plot(t,x(7:9,:))
        title('Evolution of the arm joints position', 'interpreter', ...
        'latex','fontsize',18)
        legend('$\theta_1$','$\theta_2$','$\theta_3$', 'interpreter', ...
               'latex','fontsize',18)
        xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
        ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
        grid
    end
        
    % Quadratize cost function along the trajectory
    Q = zeros(numStates,numStates,timeSteps);
    
    Q(16,16,:) = tau1c;
    Q(17,17,:) = tau2c;
    Q(18,18,:) = tau3c;
            
    Q(1,1,end) = fc;
    Q(2,2,end) = fc;
    Q(3,3,end) = fc;
    Q(4,4,end) = foc;
    Q(5,5,end) = foc;
    Q(6,6,end) = foc;
    Q(10,10,end) = fsc;
    Q(11,11,end) = fsc;
    Q(12,12,end) = fsc;
    Q(16,16,end) = tau1c;
    Q(17,17,end) = tau2c;
    Q(18,18,end) = tau3c;
    
    R = zeros(numInputs,numInputs,timeSteps);
    R(1,1,:) = ac1;
    R(2,2,:) = ac2;
    R(3,3,:) = ac3;
    
    K = zeros(numStates,numInputs,timeSteps);
        
    quadratizedCost.Q = Q;
    quadratizedCost.R = R;
    quadratizedCost.K = K;
        
    % Linearize the system dynamics and constraints along the trajectory      
    % State (x) matrix
    A = zeros(numStates,numStates,timeSteps);

    % W2EE
    A(1:6,1:6,1) = eye(6,6);
    
    % Arm joints Position
    A(7:9,7:9,1) = eye(3,3);
        
    % Arm joints acceleration
    A(13:15,10:12,1) = -1/dt*eye(3,3);
    
    % Arm joints torques
    A(16:18,13:15,1) = getB3(x(7,1), x(8,1), x(9,1));
    
%     [~,dG] = getG3(x(7,1), x(8,1), x(9,1));
%     A(16:18,7:9,1) = dG;
        
    for i = 2:timeSteps
        % W2EE
        A(1:6,1:6,i) = eye(6,6);

        % Arm joints Position
        A(7:9,7:9,i) = eye(3,3);

        % Arm joints acceleration
        A(13:15,10:12,i) = -1/dt*eye(3,3);

        % Arm joints torques
        A(16:18,13:15,i) = getB3(x(7,i-1), x(8,i-1), x(9,i-1));

%         [~,dG] = getG3(x(7,i-1), x(8,i-1), x(9,i-1));
%         A(16:18,7:9,i) = dG;
    end
    
    % Actuation (u) matrix
    B = zeros(numStates,numInputs,timeSteps);
        
    % WTEE
    B(1:6,1:3,1) = dt*jacobian3(x(7:9,1));
    
    % Arm joints Position
    B(7:9,1:3,1) = dt*eye(3,3);
    
    % Arm joints speed
    B(10:12,1:3,1) = eye(3,3);
    
    % Arm joints acceleration
    B(13:15,1:3,1) = 1/dt*eye(3,3);

    % Arm joints torques
    B(16:18,1:3,1) = getC3(x(7,1), x(8,1), x(9,1), u(1,1), u(2,1), u(3,1));
    
    for i = 2:timeSteps
        % WTEE
        B(1:6,1:3,i) = dt*jacobian3(x(7:9,i-1));

        % Arm joints Position
        B(7:9,1:3,i) = dt*eye(3,3);

        % Arm joints speed
        B(10:12,1:3,i) = eye(3,3);

        % Arm joints acceleration
        B(13:15,1:3,i) = 1/dt*eye(3,3);

        % Arm joints torques
        B(16:18,1:3,i) = getC3(x(7,i-1), x(8,i-1), x(9,i-1),...
                               u(1,i-1), u(2,i-1), u(3,i-1));
    end
    
    stateSpaceModel.A = A;
    stateSpaceModel.B = B;
            
    % Solving the inequality constrained SLQR problem
    [x, u, stateSpaceModel.I, stateSpaceModel.J, converged] = ...
       constrainedSLQ(x,x0,u,u0,dt,stateSpaceModel,quadratizedCost,config);
   
    % Check whether the algorithm has finished
    if converged > 0
        disp(['SLQ found the optimal control input within ',num2str(iter-1),' iterations'])
        break;
    end

    disp(['Iteration number ',num2str(iter),...
          ', distance to goal = ',num2str(norm(x(1:3,end)-x0(1:3,end)))]);
      
    iter = iter+1;
    if iter > maxIter
        cprintf('err','MP-FB SLQ failed to generate a motion plan\n')
        switch converged
            case 0
                error('Inequality constraints prevented SLQR convergence');
            case -1
                error('The goal was not reachable');
            case -2
                error('The SLQR algorithm failed to converge');
            case -3
                error('SLQR could not keep to the imposed constraints');
            otherwise
                error('Something unexpected prevented SLQR to converge');
        end
    end
              
end

toc

%% Plots   
iu = cumsum(abs(x(16,:))*dt);
disp(['Total torque applied arm joint 1: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(17,:))*dt);
disp(['Total torque applied arm joint 2: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(18,:))*dt);
disp(['Total torque applied arm joint 3: ',num2str(iu(end)),' Nm'])    

figure(1)
hold on;

delete(h3);
[TW0, TW1, TW2, TW3] = direct3(x(7:9,end));    
h3 = plot3([0 TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
           [0 TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
           [0 TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)],...
           'Color', [0.8 0.8 0.8], 'LineWidth', 2.5);

delete(h4);
h4 = plotFrame(TW3, 1, 0.1);  

delete(h8);
h8 = plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 5, 'Color', 'y');  

hold off;

figure(2)
plot(t,x(7:9,:))
title('Evolution of the arm joints position', 'interpreter', ...
'latex','fontsize',18)
legend('$\theta_1$','$\theta_2$','$\theta_3$', 'interpreter', ...
       'latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
grid

figure(3)
plot(t,u(1:3,:))
title('Actuating arm joints speed','interpreter','latex')
xlabel('t(s)','interpreter','latex','fontsize',18)
ylabel('$\dot\theta(rad/s$)','interpreter','latex','fontsize',18)
legend('$\dot\theta_1$','$\dot\theta_2$',...
       '$\dot\theta_3$','interpreter', ...
       'latex','fontsize',18)

% figure(4)
% plot(t,x(13:15,:))
% title('Evolution of the arm joints accelerations', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\ddot\theta_1$','$\ddot\theta_2$','$\ddot\theta_3$', 'interpreter', ...
%        'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\ddot\theta (rad/s^2)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
% figure(5)
% plot(t,x(16:18,:))
% title('Evolution of the applied arm torques', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\tau_1$','$\tau_2$','$\tau_3$', 'interpreter', ...
%        'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\tau (Nm)$', 'interpreter', 'latex','fontsize',18)
% grid


%% Simulation
% sim('manipulator3DoF',t(end));

