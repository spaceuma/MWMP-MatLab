
%% Initialization

addpath('../../../ARES-DyMu_matlab/Global Path Planning/functions')
addpath('../../maps')
addpath('../../models')
addpath('../../models/3DoF')

addpath('../../costs')
addpath('../../utils')
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

global wheelRadius;
wheelRadius = 0.2;
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

riskDistance = 1;
safetyDistance = 1.5;
mapResolution = 0.05;
vehicleSpeed = 0.1;

armJointsLimits = [-360 +360;
                   -120  80;
                   -140 +140]*pi/180;
               
wheelTorqueLimit = 2.85;

%% Constraints 
xB0 = 2.0;
yB0 = 2.5;
zB0 = zBC;
yawB0 = pi/2;

qi = [0, -pi/2, pi/2];
rollei = 0;
pitchei = pi/2;
yawei = 0;

xef = 2.1;
yef = 2.4;
zef = 0.2;
rollef = 0;
pitchef = pi;
yawef = 0;

TWB = getTraslation([xB0,yB0,zB0])*getZRot(yawB0);
[~, ~, ~, TB3] = direct3(qi);

TW3 = TWB*TB3;

xei = TW3(1,4);
yei = TW3(2,4);
zei = TW3(3,4);

%% Time horizon estimation
% FMM to compute totalCostMap
tau = 0.5; % GDM step size

load('obstMap3','obstMap')
dilatedObstMap = dilateObstMap(obstMap, riskDistance, mapResolution);
safeObstMap = dilateObstMap(obstMap, safetyDistance, mapResolution);

costMap = ones(size(obstMap));

distMap = mapResolution*bwdist(obstMap);
auxMap = 1./distMap;
gradient = 1;
minCost = max(max(costMap(costMap~=Inf)));
costMap(safeObstMap==1)= minCost + gradient*auxMap(safeObstMap==1)./min(min(auxMap(safeObstMap==1)));
costMap(safeObstMap==1)= costMap(safeObstMap==1)./min(min(costMap(safeObstMap==1)));
costMap(obstMap==1) = max(max(costMap(costMap~=Inf)));

iInit = [round(xB0/mapResolution)+1 round(yB0/mapResolution)+1];
iGoal = [round(xef/mapResolution)+1 round(yef/mapResolution)+1];

[totalCostMap, ~] = computeTmap(costMap,iGoal);

% Quadratizing totalCostMap
totalCostMap(totalCostMap == Inf) = NaN;
[gTCMx, gTCMy] = calculateGradient(mapResolution*totalCostMap);

[referencePath,~] = getPathGDM2(totalCostMap,iInit,iGoal,tau, gTCMx, gTCMy);
referencePath = (referencePath-1)*mapResolution;

while(size(referencePath,1) > 1000)
    [referencePath,~] = getPathGDM2(totalCostMap,iInit+round(2*rand(1,2)-1),iGoal,tau, gTCMx, gTCMy);
    referencePath = (referencePath-1)*mapResolution;
end

% Horizon definition
pathLength = getLength(referencePath.');
expectedTimeArrival = pathLength/vehicleSpeed;
if pathLength > 10
    warning(['The objective is too far for an efficient motion plan ',...
            'computation, the results may be unaccurate']);
elseif pathLength < reachabilityDistance
    warning(['The objective is already close to the rover, ',...
             'the expected time horizon will be set to 15 seconds']);
    expectedTimeArrival = 15;
end

% Time vector
tf = expectedTimeArrival; 
dt = tf/500;
t = 0:dt:tf;

% Initial reference path adaptation
% Resize path
x1 = 1:size(referencePath,1);
x2 = linspace(1,size(referencePath,1),size(t,2));
referencePath = interp1(x1,referencePath,x2);

yaw = getYaw(referencePath);
referencePath = [referencePath yaw].';

waypSeparation = norm(referencePath(1:2,1)-referencePath(1:2,2));

yawB0 = modulateYaw(yawB0,referencePath(3,1));
for i = 2:size(referencePath,2)
    referencePath(3,i) = modulateYaw(referencePath(3,i), referencePath(3,i-1));
end

% Obtaining gradient inside obstacles, for escaping them
[gOMxini, gOMyini] = getObstaclesEscapingGradient(safeObstMap);
gOMxini(or(isnan(gOMxini),isinf(gOMxini))) = 0;
gOMyini(or(isnan(gOMyini),isinf(gOMyini))) = 0;

h1 = fspecial('average',10);
h2 = fspecial('average',3);

gOMx = filter2(h1,gOMxini);  
gOMx = filter2(h2,gOMx);    

gOMy = filter2(h1,gOMyini);  
gOMy = filter2(h2,gOMy);

%% Costs
time_ratio = tf/60; % Ratio for the costs to ensure convergence

% State costs
fc = 1000000000/time_ratio; % Final state cost, 1000000000
foc = 0/time_ratio; % Final orientation cost, 0
fsc = 1000000/time_ratio; % Final zero speed cost, 1000000
rtc = 50/time_ratio; % Reference path max cost, 50
rtor = 0.25/time_ratio; % Percentage of rtc when wayp orientation = pi/2, 0.25
oc = 300.0/time_ratio; % Obstacles repulsive cost, 300.0
% wlc = 0.00; % Wheels torque limit cost, 0.5

tau1c = 2.0/time_ratio; % Joint 1 inverse torque constant, 2
tau2c = 2.0/time_ratio; % Joint 2 inverse torque constant, 2
tau3c = 2.0/time_ratio; % Joint 3 inverse torque constant, 2

tauWheel = 1e1*0.8/time_ratio; % Wheels joint inverse torque constant, 0.8

% Input costs
bc = 90*time_ratio; % Base actuation cost, 90
sc = 0.1*time_ratio; % Steering cost, 0.1
ac1 = 10000000*time_ratio; % Arm actuation cost, 10000000
ac2 = 10000000*time_ratio; % Arm actuation cost, 10000000
ac3 = 10000000*time_ratio; % Arm actuation cost, 10000000

% Extra costs
kappa1 = 0.02; % Influence of yaw into rover pose, tune till convergence, [0 1]
kappa2 = 0; % Influence of steer turns into final speed, tune till convergence, [0 1]

tc = 0.0; % Total cost map cost, 0.0
% tco = 0.5; % Total cost map orientation cost, 0.5

distThreshold = 0.031; % When should we stop the algorithm...? (metres)

lineSearchStep = 0.30; % Minimum actuation percentage

% iterFCApproaching = 0;

maxIter = 100; % Maximum number of iterations

%% State space model
% State vectors
numStates = 43;
x = zeros(numStates,size(t,2));
% WTEE
x(1,1) = xei;
x(2,1) = yei;
x(3,1) = zei;
% BTEE
x(4,1) = TB3(1,4);
x(5,1) = TB3(2,4);
x(6,1) = TB3(3,4);
x(7,1) = rollei;
x(8,1) = pitchei;
x(9,1) = yawei;
% WTB
x(10,1) = xB0;
x(11,1) = yB0;
x(12,1) = yawB0;
% Bspeed
x(13,1) = 0;
x(14,1) = 0;
x(15,1) = 0;
% Arm joints positions
x(16,1) = qi(1);
x(17,1) = qi(2);
x(18,1) = qi(3);
% Arm joints speeds
x(19,1) = 0;
x(20,1) = 0;
x(21,1) = 0;
% Arm joints accelerations
x(22,1) = 0;
x(23,1) = 0;
x(24,1) = 0;
% Arm joints torques
x(25,1) = 0;
x(26,1) = 0;
x(27,1) = 0;
% Wheels speed
x(28,1) = 0;
x(29,1) = 0;
x(30,1) = 0;
x(31,1) = 0;
% Wheels acceleration
x(32,1) = 0;
x(33,1) = 0;
x(34,1) = 0;
x(35,1) = 0;
% Wheels torque
x(36,1) = 0;
x(37,1) = 0;
x(38,1) = 0;
x(39,1) = 0;
% Steering joints
x(40,1) = 0;
x(41,1) = 0;
x(42,1) = 0;
x(43,1) = 0;

% Initial control law
numInputs = 7;
u = zeros(numInputs,size(t,2));

% Forward integrate system equations
x = forwardIntegrateSystem(x, u, dt);

% Target state and control trajectories
x0 = zeros(numStates,size(t,2));

x0(10:12,1:end) = referencePath;

% WTEE
x0(1,end) = xef;
x0(2,end) = yef;
x0(3,end) = zef;
% BTEE
x0(4,end) = 0;
x0(5,end) = 0;
x0(6,end) = 0;
x0(7,end) = rollef;
x0(8,end) = pitchef;
x0(9,end) = yawef;
% WTB
% x0(10,end) = 0;
% x0(11,end) = 0;
% x0(12,end) = 0;
% Bspeed
x0(13,end) = 0;
x0(14,end) = 0;
x0(15,end) = 0;
% Arm joints positions
x0(16,end) = 0;
x0(17,end) = 0;
x0(18,end) = 0;
% Arm joints speeds
x0(19,end) = 0;
x0(20,end) = 0;
x0(21,end) = 0;
% Arm joints accelerations
x0(22,end) = 0;
x0(23,end) = 0;
x0(24,end) = 0;
% Arm joints torques
x0(25,end) = 0;
x0(26,end) = 0;
x0(27,end) = 0;
% Wheels speed
x0(28,end) = 0;
x0(29,end) = 0;
x0(30,end) = 0;
x0(31,end) = 0;
% Wheels acceleration
x0(32,end) = 0;
x0(33,end) = 0;
x0(34,end) = 0;
x0(35,end) = 0;
% Wheels torque
x0(36,end) = 0;
x0(37,end) = 0;
x0(38,end) = 0;
x0(39,end) = 0;
% Steering joints
x0(40,end) = 0;
x0(41,end) = 0;
x0(42,end) = 0;
x0(43,end) = 0;

u0 = zeros(numInputs,size(t,2));

Jac = zeros(6,3,size(t,2));

% Constraints matrices definition
% State input constraints
numStateInputConstraints = 0;
I0 = zeros(numStateInputConstraints,size(t,2));
I = I0;
C = zeros(numStateInputConstraints,numStates,size(t,2));
D = zeros(numStateInputConstraints,numInputs,size(t,2));
r = zeros(numStateInputConstraints,size(t,2));

% D(1,1,:) = 1;
% r(1,:) = -FMax;
% 
% D(2,1,:) = -1;
% r(2,:) = FMin;

% Pure state constraints
numPureStateConstraints = 0;
J0 = zeros(numPureStateConstraints,size(t,2));
J = J0;
G = zeros(numPureStateConstraints,numStates,size(t,2));
h = zeros(numPureStateConstraints,size(t,2));

% G(1,36,:) = 1;
% h(1,:) = -wheelTorqueLimit;
% 
% G(2,37,:) = 1;
% h(2,:) = -wheelTorqueLimit;
% 
% G(3,38,:) = 1;
% h(3,:) = -wheelTorqueLimit;
% 
% G(4,39,:) = 1;
% h(4,:) = -wheelTorqueLimit;
% 
% G(5,36,:) = -1;
% h(5,:) = -wheelTorqueLimit;
% 
% G(6,37,:) = -1;
% h(6,:) = -wheelTorqueLimit;
% 
% G(7,38,:) = -1;
% h(7,:) = -wheelTorqueLimit;
% 
% G(8,39,:) = -1;
% h(8,:) = -wheelTorqueLimit;

Jac = zeros(6,3,size(t,2));

% % Plotting stuff
map = [0 0.6   0
       0.6 0.3 0
       0.6 0   0];
colormap(map);
xVect = linspace(0,9.95,200);
[X,Y] = meshgrid(xVect,xVect);

%% SLQR algorithm
iter = 1;
while 1   
    
    figure(1)
    % Plotting first arm config
    [TB0, TB1, TB2, TB3] = direct3(x(16:18,1));
    TWB = getTraslation([x(10,1),x(11,1),zBC])*getZRot(x(12,1));
    TW0 = TWB*TB0;
    TW1 = TWB*TB1;
    TW2 = TWB*TB2;
    TW3 = TWB*TB3;
    plot3([TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
          [TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
          [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)], 'Color', 'r', 'LineWidth', 2.5);
    hold on;
    
    % Plotting last arm config
    [TB0, TB1, TB2, TB3] = direct3(x(16:18,end-1));
    TWB = getTraslation([x(10,end-1),x(11,end-1),zBC])*getZRot(x(12,end-1));
    TW0 = TWB*TB0;
    TW1 = TWB*TB1;
    TW2 = TWB*TB2;
    TW3 = TWB*TB3;
    plot3([TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
          [TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
          [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)], 'Color', 'r', 'LineWidth', 2.5);
      
    % Plotting first rover position
    TWB = getTraslation([x(10,1),x(11,1),zBC])*getZRot(x(12,1));
    TB1 = getTraslation([dfy,dfx,-zBC]);
    TB2 = getTraslation([-dfy,dfx,-zBC]);
    TB3 = getTraslation([-dfy,-dfx,-zBC]);
    TB4 = getTraslation([dfy,-dfx,-zBC]);
    TW1 = TWB*TB1;
    TW2 = TWB*TB2;
    TW3 = TWB*TB3;
    TW4 = TWB*TB4;
    plot3([TWB(1,4) TW1(1,4) TWB(1,4) TW2(1,4) TWB(1,4) TW3(1,4) TWB(1,4) TW4(1,4)],...
          [TWB(2,4) TW1(2,4) TWB(2,4) TW2(2,4) TWB(2,4) TW3(2,4) TWB(2,4) TW4(2,4)],...
          [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', 'r', 'LineWidth', 2.5);
    hold on;
   
    quiver3(TWB(1,4), TWB(2,4), TWB(3,4), cos(x(12,1))/2, sin(x(12,1))/2, 0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.7)
    quiver3(TWB(1,4), TWB(2,4), TWB(3,4), -sin(x(12,1))/2, cos(x(12,1))/2, 0, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.7)
    quiver3(TWB(1,4), TWB(2,4), TWB(3,4), 0, 0, 1/2, 'Color', 'c', 'LineWidth', 2, 'MaxHeadSize', 0.7) 
    
    % Plotting last rover position
    TWB = getTraslation([x(10,end-1),x(11,end-1),zBC])*getZRot(x(12,end-1));
    TW1 = TWB*TB1;
    TW2 = TWB*TB2;
    TW3 = TWB*TB3;
    TW4 = TWB*TB4;
    plot3([TWB(1,4) TW1(1,4) TWB(1,4) TW2(1,4) TWB(1,4) TW3(1,4) TWB(1,4) TW4(1,4)],...
              [TWB(2,4) TW1(2,4) TWB(2,4) TW2(2,4) TWB(2,4) TW3(2,4) TWB(2,4) TW4(2,4)],...
              [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', 'r', 'LineWidth', 2.5);

    quiver3(TWB(1,4), TWB(2,4), TWB(3,4), cos(x(12,end-1))/2, sin(x(12,end-1))/2, 0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.7)
    quiver3(TWB(1,4), TWB(2,4), TWB(3,4), -sin(x(12,end-1))/2, cos(x(12,end-1))/2, 0, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.7)
    quiver3(TWB(1,4), TWB(2,4), TWB(3,4), 0, 0, 1/2, 'Color', 'c', 'LineWidth', 2, 'MaxHeadSize', 0.7)
      
    % Plotting scenario
    daspect([1 1 1])
    contourf(X,Y,dilatedObstMap+obstMap);
    plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 5, 'Color', 'y')
    plot3(x(10,1:end-1),x(11,1:end-1),zBC*ones(size(x,2)-1), 'LineWidth', 5, 'Color', [1,0.5,0])
    title('Mobile manipulator trajectories', 'interpreter', ...
    'latex','fontsize',18)
    plot3(x0(10,:),x0(11,:), zBC*ones(size(t,2),2), 'LineWidth', 5, 'Color', [0,0,0.6])
    plot3(referencePath(1,:),referencePath(2,:), zBC*ones(size(t,2),2), 'LineWidth', 5, 'Color', [0,0,1])
    plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', 'c')

    hold off;    
    
    figure(8)
    plot(t,x(36:39,:))
    hold on
    yline(wheelTorqueLimit,'--');
    yline(-wheelTorqueLimit,'--');
    title('Evolution of the applied wheel torques', 'interpreter', ...
    'latex','fontsize',18)
    legend('$\tau_{\omega 1}$','$\tau_{\omega 2}$','$\tau_{\omega 3}$',...
            '$\tau_{\omega 4}$', 'interpreter','latex','fontsize',18)
    xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
    ylabel('$\tau (Nm)$', 'interpreter', 'latex','fontsize',18)
    grid
    hold off

    % Multitrajectory costing method
    % Filtering undesired updates
    for i = 2:ceil(size(t,2)/50):size(t,2)-2
        d = norm(x(10:11,i)-x0(10:11,i));
        if d < 100*waypSeparation && d > 50*waypSeparation && ...
           isSafePath(x(10,:),x(11,:), mapResolution,dilatedObstMap)
            % Obtaining the candidate path
            iInit = [round(x(10,i)/mapResolution)+1 round(x(11,i)/mapResolution)+1];
            if(iInit(1)>size(totalCostMap,1)-2)
                iInit(1) = size(totalCostMap,1)-2;
            end
            if(iInit(1)<3)
                iInit(1) = 3;
            end
            if(iInit(2)>size(totalCostMap,2)-2)
                iInit(2) = size(totalCostMap,2)-2;
            end
            if(iInit(2)<3)
                iInit(2) = 3;
            end

            [pathi,~] = getPathGDM2(totalCostMap,iInit,iGoal,tau, gTCMx, gTCMy);
            pathi = (pathi-1)*mapResolution;

            while(size(pathi,1) > 1000)
                [pathi,~] = getPathGDM2(totalCostMap,iInit+round(2*rand(1,2)-1),iGoal,tau, gTCMx, gTCMy);
                pathi = (pathi-1)*mapResolution;
            end
            

            yaw = getYaw(pathi);
            pathi = [pathi yaw].';
            
            % Compute switch waypoint
            [switchPose, switchIndex] = getSwitch(x0(10:12,:), pathi, i);
            % The above option is computionally too expensive
%             switchIndex = i;
%             switchPose = x0(1:2,i);

            % Compute intersection waypoint
            [inter, interIndex1, interIndex2] = getIntesection(pathi, x0(10:12,switchIndex:end), waypSeparation);
            interIndex2 = interIndex2 + switchIndex - 1;
                        
            % If the candidate path is big enough...
            if(interIndex1 > 1 && interIndex2 ~= switchIndex)
                pathCandidate = pathi(:,1:interIndex1);
                originalPath = x0(10:12,switchIndex:interIndex2);

                % Resize path
                x1 = 1:size(pathCandidate,2);
                x2 = linspace(1,size(pathCandidate,2),size(originalPath,2));
                pathCandidate = (interp1(x1,pathCandidate.',x2)).';

                % Get path costs (candidate and original)
                newCost = getTrajectoryCost(pathCandidate, x(10:12,:), switchIndex);
                oldCost = getTrajectoryCost(originalPath, x(10:12,:), switchIndex);

                % If the cost is reduced, the candidate is safe and the
                % change is significant, update the reference path
                if newCost*1.05 < oldCost && ...
                   isSafePath([x(10,1:switchIndex-1) pathCandidate(1,:) x(10,interIndex2+1:end)],...
                              [x(11,1:switchIndex-1) pathCandidate(2,:) x(11,interIndex2+1:end)],...
                              mapResolution,dilatedObstMap) && ...
                              DiscreteFrechetDist(pathCandidate, originalPath) > 2*waypSeparation
                    x0(10:12,switchIndex:interIndex2) = pathCandidate;
                    for j = switchIndex:interIndex2
                        if j > 1
                            x0(12,j) = modulateYaw(x0(12,j), x0(12,j-1));
                        end
                    end
                    disp(['Changing reference path from waypoint ',num2str(switchIndex), ' to ',num2str(interIndex2)])
                end
            end
        end

    end  
    
    xs = zeros(numStates,size(t,2));
    us = zeros(numInputs,size(t,2));
    
    % Quadratize cost function along the trajectory
    Q = zeros(numStates,numStates,size(t,2));

    for i = 1:size(t,2)
        Q(10:12,10:12,i) = eye(3,3)*rtc;
        
        if norm([x(10,i) x(11,i)] - [xef yef]) < reachabilityDistance
            Q(10:12,10:12,i) = Q(10:12,10:12,i)/9999;
        end
    end  
    
    Q(25,25,:) = tau1c;
    Q(26,26,:) = tau2c;
    Q(27,27,:) = tau3c;
    
    Q(36,36,:) = tauWheel;
    Q(37,37,:) = tauWheel;
    Q(38,38,:) = tauWheel;
    Q(39,39,:) = tauWheel;
       
    Q(1,1,end) = fc;
    Q(2,2,end) = fc;
    Q(3,3,end) = fc;
    Q(7,7,end) = foc;
    Q(8,8,end) = foc;
    Q(9,9,end) = foc;
    Q(13,13,end) = fsc;
    Q(14,14,end) = fsc;
    Q(15,15,end) = fsc;
    
    R = zeros(numInputs,numInputs,size(t,2));
    R(1,1,:) = ac1;
    R(2,2,:) = ac2;
    R(3,3,:) = ac3;
    R(4,4,:) = bc;
    R(5,5,:) = bc;  
    R(6,6,:) = sc;
    R(7,7,:) = sc; 
    
    K = zeros(numStates,numInputs,size(t,2));

    % Define the sequential state and input vectors
    xs0 = zeros(numStates,size(t,2));
    us0 = zeros(numInputs,size(t,2));

    for i = 1:size(t,2)
        xs0(:,i) = Q(:,:,i)*(x(:,i) - x0(:,i));
        us0(:,i) = R(:,:,i)*(u(:,i) - u0(:,i));
        
        % Según Sideris, debería ser como sigue, aunque con esto no estamos
        % realimentando el vector de referencia
%         xs0(:,i) = Q(:,:,i)*x(:,i) + x0(:,i);
%         us0(:,i) = R(:,:,i)*u(:,i) + u0(:,i);
    end

    % Linearize the system dynamics and constraints along the trajectory  
    for i = 2:size(t,2)
        Jac(:,:,i-1) = jacobian3(x(16:18,i-1));
    end
    
     % State (x) matrix
    A = zeros(numStates,numStates,size(t,2));

    % W2EEx
    A(1,4,1) = cos(x(12,1));
    A(1,5,1) = -sin(x(12,1));
    A(1,10,1) = 1;

    % W2EEy
    A(2,4,1) = sin(x(12,1));
    A(2,5,1) = cos(x(12,1));
    A(2,11,1) = 1;

    % W2EEz
    A(3,3,1) = 1;

    % B2EE
    A(4:9,4:9,1) = eye(6,6);

    % W2Bx
    A(10,10,1) = 1;
    A(10,12,1) = dt*(-sin(x(12,1))*x(13,1)*kappa1-cos(x(12,1))*x(14,1)*kappa1);
    A(10,13,1) = dt*(cos(x(12,1))+sin(x(12,1))*x(12,1)*kappa1);
    A(10,14,1) = -dt*(sin(x(12,1))-cos(x(12,1))*x(12,1)*kappa1);

    % W2By
    A(11,11,1) = 1;
    A(11,12,1) = dt*(cos(x(12,1))*x(13,1)*kappa1-sin(x(12,1))*x(14,1)*kappa1);
    A(11,13,1) = dt*(sin(x(12,1))-cos(x(12,1))*x(12,1)*kappa1);
    A(11,14,1) = dt*(cos(x(12,1))+sin(x(12,1))*x(12,1)*kappa1);

    % W2B Heading
    A(12,12,1) = 1;
    A(12,15,1) = dt;

    % W2B Speed x
    A(13,40,1) = -wheelRadius/2*sin(x(40,1))*u(4,1)*kappa2;
    A(13,42,1) = -wheelRadius/2*sin(x(42,1))*u(5,1)*kappa2;
    
    % W2B Speed y
    A(14,40,1) = -wheelRadius/2*cos(x(40,1))*u(4,1)*kappa2;
    A(14,42,1) = -wheelRadius/2*cos(x(42,1))*u(5,1)*kappa2;
    
    % W2B Speed Heading
    A(15,40,1) = wheelRadius/(2*dfx) *(-sin(x(40,1))*u(4,1))*kappa2;
    A(15,42,1) = -wheelRadius/(2*dfx)*(-sin(x(42,1))*u(5,1))*kappa2;
    
    % Arm joints Position
    A(16:18,16:18,1) = eye(3,3);
        
    % Arm joints acceleration
    A(22:24,19:21,1) = -1/dt*eye(3,3);
    
    % Arm joints torques
    A(25:27,22:24,1) = getB3(x(16,1), x(17,1), x(18,1));
    
    [~,dG] = getG3(x(16,1), x(17,1), x(18,1));
    A(25:27,16:18,1) = dG;
    
    % Wheels accelerations
    A(32:35,28:31,1) = -eye(4,4)/dt;
    
    % Wheels torques
    A(36:39,32:35,1) = eye(4,4)*(getWheelInertia(wheelMass,wheelRadius)+...
                                 vehicleMass/4*wheelRadius*wheelRadius);
    
    % Steering Joints Position
    A(40:43,40:43,1) = eye(4,4);

    
    for i = 2:size(t,2)
        % W2EEx
        A(1,4,i) = cos(x(12,i-1));
        A(1,5,i) = -sin(x(12,i-1));
        A(1,10,i) = 1;

        % W2EEy
        A(2,4,i) = sin(x(12,i-1));
        A(2,5,i) = cos(x(12,i-1));
        A(2,11,i) = 1;

        % W2EEz
        A(3,3,i) = 1;

        % B2EE
        A(4:9,4:9,i) = eye(6,6);

        % W2Bx
        A(10,10,i) = 1;
        A(10,12,i) = dt*(-sin(x(12,i-1))*x(13,i-1)*kappa1-cos(x(12,i-1))*x(14,i-1)*kappa1);
        A(10,13,i) = dt*(cos(x(12,i-1))+sin(x(12,i-1))*x(12,i-1)*kappa1);
        A(10,14,i) = -dt*(sin(x(12,i-1))-cos(x(12,i-1))*x(12,i-1)*kappa1);

        % W2By
        A(11,11,i) = 1;
        A(11,12,i) = dt*(cos(x(12,i-1))*x(13,i-1)*kappa1-sin(x(12,i-1))*x(14,i-1)*kappa1);
        A(11,13,i) = dt*(sin(x(12,i-1))-cos(x(12,i-1))*x(12,i-1)*kappa1);
        A(11,14,i) = dt*(cos(x(12,i-1))+sin(x(12,i-1))*x(12,i-1)*kappa1);

        % W2B Heading
        A(12,12,i) = 1;
        A(12,15,i) = dt;

        % W2B Speed x
        A(13,40,i) = -wheelRadius/2*sin(x(40,i-1))*u(4,i-1)*kappa2;
        A(13,42,i) = -wheelRadius/2*sin(x(42,i-1))*u(5,i-1)*kappa2;

        % W2B Speed y
        A(14,40,i) = -wheelRadius/2*cos(x(40,i-1))*u(4,i-1)*kappa2;
        A(14,42,i) = -wheelRadius/2*cos(x(42,i-1))*u(5,i-1)*kappa2;

        % W2B Speed Heading
        A(15,40,i) = wheelRadius/(2*dfx) *(-sin(x(40,i-1))*u(4,i-1))*kappa2;
        A(15,42,i) = -wheelRadius/(2*dfx)*(-sin(x(42,i-1))*u(5,i-1))*kappa2;

        % Arm Joints Position
        A(16:18,16:18,i) = eye(3,3);
        
        % Arm joints acceleration
        A(22:24,19:21,i) = -1/dt*eye(3,3);
        
        % Arm joints torques
        A(25:27,22:24,i) = getB3(x(16,i-1), x(17,i-1), x(18,i-1));
        
        [~,dG] = getG3(x(16,i-1), x(17,i-1), x(18,i-1));
        A(25:27,16:18,i) = dG;
        
        % Wheels accelerations
        A(32:35,28:31,i) = -eye(4,4)/dt;

        % Wheels torques
        A(36:39,32:35,i) = eye(4,4)*(getWheelInertia(wheelMass,wheelRadius)+vehicleMass/4*wheelRadius*wheelRadius);
        
        % Steering Joints Position
        A(40:43,40:43,i) = eye(4,4);

    end
    
    % Actuation (u) matrix
    B = zeros(numStates,numInputs,size(t,2));
    
    % WTEEz
    B(3,1:3,1) = dt*Jac(3,:,1);
    
    % BTEE
    B(4:9,1:3,1) = dt*Jac(:,:,1);

    % W2B Speed x
    B(13,4,1) = wheelRadius/2*(cos(x(40,1)) + x(40,1)*sin(x(40,1))*kappa2);
    B(13,5,1) = wheelRadius/2*(cos(x(42,1)) + x(42,1)*sin(x(42,1))*kappa2);
    
    % W2B Speed y
    B(14,4,1) = -wheelRadius/2*(sin(x(40,1)) - x(40,1)*cos(x(40,1))*kappa2);
    B(14,5,1) = -wheelRadius/2*(sin(x(42,1)) - x(42,1)*cos(x(42,1))*kappa2);
    
    % W2B Speed heading
    B(15,4,1) = wheelRadius/(2*dfx)*(cos(x(40,1)) + x(40,1)*sin(x(40,1))*kappa2);
    B(15,5,1) = -wheelRadius/(2*dfx)*(cos(x(42,1)) + x(42,1)*sin(x(42,1))*kappa2);
    
    % Arm joints Position
    B(16:18,1:3,1) = dt*eye(3,3);
    
    % Arm joints speed
    B(19:21,1:3,1) = eye(3,3);
    
    % Arm joints acceleration
    B(22:24,1:3,1) = 1/dt*eye(3,3);

    % Arm joints torques
    B(25:27,1:3,1) = getC3(x(16,1), x(17,1), x(18,1), u(1,1), u(2,1), u(3,1));
    
    % Wheels speeds
    B(28:29,4,1) = 1;
    B(30:31,5,1) = 1;
    
    % Wheels acceleration
    B(32:33,4,1) = 1/dt;
    B(34:35,5,1) = 1/dt;
    
    % Steering Joints Position
    B(40:41,6,1) = dt;
    B(42:43,7,1) = dt;

    for i = 2:size(t,2)
        % WTEEz
        B(3,1:3,i) = dt*Jac(3,:,i-1);
        
        % BTEE
        B(4:9,1:3,i) = dt*Jac(:,:,i-1);

        % W2B Speed x
        B(13,4,i) = wheelRadius/2*(cos(x(40,i-1)) + x(40,i-1)*sin(x(40,i-1))*kappa2);
        B(13,5,i) = wheelRadius/2*(cos(x(42,i-1)) + x(42,i-1)*sin(x(42,i-1))*kappa2);

        % W2B Speed y
        B(14,4,i) = -wheelRadius/2*(sin(x(40,i-1)) - x(40,i-1)*cos(x(40,i-1))*kappa2);
        B(14,5,i) = -wheelRadius/2*(sin(x(42,i-1)) - x(42,i-1)*cos(x(42,i-1))*kappa2);

        % W2B Speed heading
        B(15,4,i) = wheelRadius/(2*dfx)*(cos(x(40,i-1)) + x(40,i-1)*sin(x(40,i-1))*kappa2);
        B(15,5,i) = -wheelRadius/(2*dfx)*(cos(x(42,i-1)) + x(42,i-1)*sin(x(42,i-1))*kappa2);

        % Arm Joints Position
        B(16:18,1:3,i) = dt*eye(3,3);
        
        % Arm joints speed
        B(19:21,1:3,i) = eye(3,3);
        
        % Arm joints acceleration
        B(22:24,1:3,i) = 1/dt*eye(3,3);

        % Arm joints torques
        B(25:27,1:3,i) = getC3(x(16,i-1), x(17,i-1), x(18,i-1), u(1,i-1), u(2,i-1), u(3,i-1));
        
        % Wheels speeds
        B(28:29,4,i) = 1;
        B(30:31,5,i) = 1;

        % Wheels acceleration
        B(32:33,4,i) = 1/dt;
        B(34:35,5,i) = 1/dt;
        
        % Steering Joints Position
        B(40:41,6,i) = dt;
        B(42:43,7,i) = dt;

    end
    
    % Active Constraints definition
    % State input constraints
    tl = [];
    p = cell(size(t,2),1);
    paux = zeros(numStateInputConstraints,1);
    pl = zeros(numStateInputConstraints,1);
    
    for i = 1:size(t,2)
        anyActiveConstraint = false;
        for j = 1:numStateInputConstraints
            if I(j,i)
                p{i} = [p{i}; j];
                paux(j) = 1;
                anyActiveConstraint = true;
            end
        end
        if anyActiveConstraint
            tl = [tl i];
        end
    end

    numActiveSIConstraints = sum(paux);
    lp = zeros(numActiveSIConstraints,1);

    index = 1;
    for i = 1:numStateInputConstraints
        if paux(i)
            pl(i) = index;
            lp(index) = i;
            index = index+1;
        end
    end
    
    Cl = zeros(numActiveSIConstraints,numStates,size(t,2));
    Dl = zeros(numActiveSIConstraints,numInputs,size(t,2));
    rl = zeros(numActiveSIConstraints,size(t,2));
    
    for i = tl
        Cl(pl(p{i}),:,i) = C(p{i},:,i);
        Dl(pl(p{i}),:,i) = D(p{i},:,i);
%         rl(pl(p{i}),i) = r(p{i},i);
    end
    
    
    % Pure state constraints
    tk = [];
    q = cell(size(t,2),1);
    qaux = zeros(numPureStateConstraints,1);
    ql = zeros(numPureStateConstraints,1);

    for i = 1:size(t,2)
        anyActiveConstraint = false;
        for j = 1:numPureStateConstraints
            if J(j,i)
                q{i} = [q{i}; j];
                qaux(j) = 1;
                anyActiveConstraint = true;
            end
        end
        if anyActiveConstraint
            tk = [tk i];
        end
    end

    numActivePSConstraints = sum(qaux);
    lq = zeros(numActivePSConstraints,1);
    
    index = 1;
    for i = 1:numPureStateConstraints
        if qaux(i)
            ql(i) = index;
            lq(index) = i;
            index = index+1;
        end
    end
    
    
    Gk = zeros(numActivePSConstraints,numStates,size(t,2));
    hk = zeros(numActivePSConstraints,size(t,2));
    
    for i = tk
        Gk(ql(q{i}),:,i) = G(q{i},:,i);
%         hk(ql(q{i}),i) = h(q{i},i);
    end
    
    % Predefinitions
    Dh = zeros(numActiveSIConstraints,numActiveSIConstraints,size(t,2));
    E = zeros(numActiveSIConstraints,numStates,size(t,2));
    rh = zeros(numActiveSIConstraints,size(t,2));
    
    Ah = zeros(numStates,numStates,size(t,2));
    Rh = zeros(numStates,numStates,size(t,2));
    Qh = zeros(numStates,numStates,size(t,2));
    x0h = zeros(numStates,size(t,2));
    u0h = zeros(numStates,size(t,2));
    
    for i = tl
        Dh(pl(p{i}),pl(p{i}),i) = inv(Dl(pl(p{i}),:,i)/R(:,:,i)*Dl(pl(p{i}),:,i).');
    end
    
    for i = 1:size(t,2)

        E(:,:,i) = Cl(:,:,i) - Dl(:,:,i)/R(:,:,i)*K(:,:,i).';
        rh(:,i) = rl(:,i) - Dl(:,:,i)/R(:,:,i)*us0(:,i);
        
        Ah(:,:,i) = A(:,:,i) - B(:,:,i)/R(:,:,i)*(K(:,:,i).' + Dl(:,:,i).'*Dh(:,:,i)*E(:,:,i));
        Rh(:,:,i) = B(:,:,i)/R(:,:,i)*(eye(numInputs,numInputs)-Dl(:,:,i).'*Dh(:,:,i)*Dl(:,:,i)/R(:,:,i))*B(:,:,i).';
        Qh(:,:,i) = Q(:,:,i) - K(:,:,i)/R(:,:,i)*K(:,:,i).' + E(:,:,i).'*Dh(:,:,i)*E(:,:,i);
        x0h(:,i) = xs0(:,i) - K(:,:,i)/R(:,:,i)*us0(:,i) + E(:,:,i).'*Dh(:,:,i)*rh(:,i);
        u0h(:,i) = -B(:,:,i)/R(:,:,i)*(us0(:,i) + Dl(:,:,i).'*Dh(:,:,i)*rh(:,i));
    end
            
    % Obstacles limits cost
    Ox = zeros(numStates,size(t,2));
    for i = 1:size(t,2)-1
        [Ox(10,i), Ox(11,i)] = getGradientTotalCost(x(10,i), x(11,i), mapResolution, gOMx, gOMy);
        Ox(10,i) = oc*Ox(10,i);
        Ox(11,i) = oc*Ox(11,i);
    end

    % LQ problem solution
    M = zeros(numStates,numStates,size(t,2));
    P = zeros(numStates,numStates,size(t,2));
    z = zeros(numStates,size(t,2));
    P(:,:,end) = Q(:,:,end);
    z(:,end) = xs0(:,end) + Ox(:,end);
    
    % Solve backward
    for i = size(t,2)-1:-1:1
        M(:,:,i) = inv(eye(numStates) + Rh(:,:,i)*P(:,:,i+1));
        P(:,:,i) = Qh(:,:,i) + Ah(:,:,i).'*P(:,:,i+1)*M(:,:,i)*Ah(:,:,i);
        z(:,i) = Ah(:,:,i).'*M(:,:,i).'*z(:,i+1) + ...
            Ah(:,:,i).'*P(:,:,i+1)*M(:,:,i)*u0h(:,i) + x0h(:,i) + Ox(:,i);
    end
    
    Gamma = zeros(numActivePSConstraints*size(tk,2),numStates);
    Gammak = zeros(numActivePSConstraints,numStates,size(t,2), size(tk,2));
    y = zeros(numActivePSConstraints*size(tk,2),1);
    F = zeros(numActivePSConstraints*size(tk,2),numActivePSConstraints*size(tk,2));
    H = zeros(numActivePSConstraints*size(tk,2),1);

    % Solve over all state constraints   
    for k = 1:size(tk,2)
        yk = zeros(numActivePSConstraints,tk(k));

        Gammak(:,:,tk(k),k) = Gk(:,:,tk(k));

        for n = tk(k)-1:-1:1
            Gammak(:,:,n,k) = Gammak(:,:,n+1,k)*M(:,:,n)*Ah(:,:,n);
            yk(:,n) = yk(:,n+1) + Gammak(:,:,n+1,k)*M(:,:,n)*(u0h(:,n) - Rh(:,:,n) * z(:,n+1));
        end

        H((k-1)*numActivePSConstraints+1:k*numActivePSConstraints) = hk(:,tk(k));
        Gamma((k-1)*numActivePSConstraints+1:k*numActivePSConstraints,:) = Gammak(:,:,1,k);
        y((k-1)*numActivePSConstraints+1:k*numActivePSConstraints) = yk(:,1);        

    end
    for k = 1:size(tk,2)
        for j = 1:size(tk,2)
            minConstraintIndex = min(tk(k),tk(j));
            Fkj = zeros(numActivePSConstraints,numActivePSConstraints,minConstraintIndex);
            for n = minConstraintIndex-1:-1:1
                Fkj(:,:,n) = Fkj(:,:,n+1) - Gammak(:,:,n+1,k)*M(:,:,n)*Rh(:,:,n)*Gammak(:,:,n+1,j).';
            end
            F((k-1)*numActivePSConstraints+1:k*numActivePSConstraints,...
              (j-1)*numActivePSConstraints+1:j*numActivePSConstraints) = Fkj(:,:,1);
        end
    end
    
    nuV = zeros(numActivePSConstraints*size(tk,2),1);
    
    % Ensuring correct inverse matrix computation
    if(~det(F))
        cprintf('SystemCommands','WARNING: Cannot compute inverse, determinant is 0\n')
        correctIndex = [];        
        for k = 1:size(tk,2)
            correctIndex = [correctIndex (k-1)*numActivePSConstraints+ql(q{tk(k)}).'];
        end
        Faux = F(correctIndex,correctIndex);
        if (rank(Faux) < size(Faux,2))
            cprintf('SystemCommands','WARNING: Cannot compute inverse, linearly dependant terms in matrix Faux\n')
            [Ffraux, indRemoved, indKeeped, indSimilar] = getFullRankMatrix(Faux);
            nuV(correctIndex(indKeeped)) = Ffraux\...
                (-Gamma(correctIndex(indKeeped),:)*xs(:,1) - ...
                y(correctIndex(indKeeped)) - H(correctIndex(indKeeped)));
            
            nuV(correctIndex(indRemoved)) = nuV(correctIndex(indSimilar));

        else
            invFaux = inv(Faux);
            invF(correctIndex,correctIndex) = invFaux; 
            nuV(:) = invF*(-Gamma*xs(:,1) - y - H);
        end
    elseif (rank(F) < numActivePSConstraints*size(tk,2))
        cprintf('SystemCommands','WARNING: Cannot compute inverse, linearly dependant terms in matrix F\n')
        [Ffr, indRemoved, indKeeped, indSimilar] = getFullRankMatrix(F);
        nuV(indKeeped) = Ffr\(-Gamma(indKeeped,:)*xs(:,1) - y(indKeeped) - H(indKeeped));
        nuV(indRemoved) = nuV(indSimilar);
    else
        nuV(:) = F\(-Gamma*xs(:,1) - y - H);
    end
    
    
    nu = zeros(numActivePSConstraints,size(t,2));
    for i = 1:size(tk,2)
        nu(:,tk(i)) = nuV((i-1)*numActivePSConstraints+1:i*numActivePSConstraints);
    end   
    
    s = zeros(numStates,size(t,2));
    if numActivePSConstraints
        for i = 1:size(t,2)
            sumG = 0;
            for k = 1:size(tk,2)
                if (tk(k) >= i)
                    sumG = sumG + Gammak(:,:,i,k).'*nu(:,tk(k));
                end
            end
            s(:,i) = z(:,i) + sumG;
        end
    else
        s(:,:) = z(:,:);
    end
        
    v = zeros(numStates,size(t,2));
    lambda = zeros(numStates,size(t,2));
    mu = zeros(numActiveSIConstraints,size(t,2));

    % Solve forward
    for i = 1:size(t,2)-1
        v(:,i) = M(:,:,i)*(u0h(:,i) - Rh(:,:,i)*s(:,i+1));
        xs(:,i+1) = M(:,:,i)*Ah(:,:,i)*xs(:,i) + v(:,i);
        lambda(:,i+1) = P(:,:,i+1)*xs(:,i+1) + s(:,i+1);
        mu(:,i) = Dh(:,:,i)*(E(:,:,i)*xs(:,i) - Dl(:,:,i)/R(:,:,i)*B(:,:,i).'*lambda(:,i+1) + rh(:,i));
        us(:,i) = -R(:,:,i)\(K(:,:,i).'*xs(:,i) + B(:,:,i).'*lambda(:,i+1) + Dl(:,:,i).'*mu(:,i) + us0(:,i));
    end
    iter = iter+1;

    step3 = true;
    if norm(us)>=0.0001*norm(u)
        % Step 2
        rhoi = ones(numStateInputConstraints,size(t,2));
        deltai = ones(numStateInputConstraints,size(t,2));
        
        rhoj = ones(numPureStateConstraints,size(t,2));
        deltaj = ones(numPureStateConstraints,size(t,2));

        for n = 1:size(t,2)
            for i = 1:numStateInputConstraints
                if(~I(i,n))
                    rhoi(i,n) = C(i,:,n)*x(:,n) + D(i,:,n)*u(:,n) + r(i,n);
                    deltai(i,n) = C(i,:,n)*xs(:,n) + D(i,:,n)*us(:,n);
                end
            end
            for j = 1:numPureStateConstraints
                if(~J(j,n))
                    rhoj(j,n) = G(j,:,n)*x(:,n) + h(j,n);
                    deltaj(j,n) = G(j,:,n)*xs(:,n);
                end
            end
        end
              
        thetak = min(-rhoi(~I & deltai>0)./deltai(~I & deltai>0));
        betak = min(-rhoj(~J & deltaj>0)./deltaj(~J & deltaj>0));

        alfak = min([1 thetak betak]);      
%         alfak = 1;
        
        if alfak == 1
            step3 = true;
            
            % Line search to optimize alfa
            alfa = 1:-lineSearchStep:0.0001;
            Jcost = zeros(size(alfa));

            xk = x;
            uk = u;
            for n = 1:size(alfa,2)
                u = uk + alfa(n)*us;
                
%                 x = xk + alfa(n)*xs;
                x = forwardIntegrateSystem(x, u, dt);

                Jcost(n) = 1/2*(x(:,end)-x0(:,end)).'*Q(:,:,end)*(x(:,end)-x0(:,end))...
                    + 100*~isSafePath(x(1,:),x(2,:),mapResolution,dilatedObstMap);
                for i = 1:size(t,2)-1
                    Jcost(n) = Jcost(n) + 1/2*((x(:,i)-x0(:,i)).'*Q(:,:,i)*(x(:,i)-x0(:,i))...
                        + (u(:,i)-u0(:,i)).'*R(:,:,i)*(u(:,i)-u0(:,i)));
                end            
            end
            [mincost, ind] = min(Jcost);
            alfamin = alfa(ind);

            % Update controller
            u = uk + alfamin*us;
            
%             x = xk + alfamin*xs;
            x = forwardIntegrateSystem(x, u, dt);

            
        else
            step3 = false;
            
            % Update controller
            u = u + alfak*us;
            
%             x = x + alfak*xs;
            x = forwardIntegrateSystem(x,u);

            
            for n = 1:size(t,2)
                for i = 1:numStateInputConstraints
                    if(-rhoi(i,n)/deltai(i,n) == alfak && ~I(i,n) && n < size(t,2))
                        I(i,n) = 1;
                    end
                end
                for j = 1:numPureStateConstraints
                    if(-rhoj(j,n)/deltaj(j,n) == alfak && ~J(j,n)&& n > 1)                        
                        J(j,n) = 1;
                    end
                end
            end
        end
    end
    
    endDist = norm(x(1:3,end)-x0(1:3,end));
    if step3        
        % Step 3
        if size(tl,2) > 0
            minMu = zeros(1,size(t,2));
            iS = zeros(1,size(t,2));

            for m = tl
                minMu(m) = 99999999;
                for i = 1:size(p{m},1)
                    if I(p{m}(i),m) && ~I0(p{m}(i),m) && mu(i,m) < minMu(m)
                        minMu(m) = mu(i,m);
                        iS(m) = i;
                    end
                end
                if minMu(m) == 99999999
                    minMu(m) = 0;
                end
            end
            
            [minimumMu, maux] = min(minMu(1,tl));
            mS = tl(maux);
        else
            minimumMu = 0;
            mS = 1;
            iS = 1;
        end

        if size(tk,2) > 0
            minNu = zeros(1,size(t,2));
            jS = zeros(1,size(t,2));

            for l = tk
                minNu(1,l) = 99999999;
                for j = 1:size(q{l},1)
                    if J(q{l}(j),l) && ~J0(q{l}(j),l) && nu(j,l) < minNu(1,l)
                        minNu(1,l) = nu(j,l);
                        jS(l) = j;
                    end
                end
                if minNu(1,l) == 99999999
                    minNu(1,l) = 0;
                end
            end
            [minimumNu, laux] = min(minNu(1,tk));
            lS = tk(laux);

        else
            minimumNu = 0;
            lS = 1;
            jS = 1;
        end        
        
        % Exit condition
        if minimumMu >= -1e-5 && minimumNu >=-1e-5 && ...
          (isSafePath(x(10,:),x(11,:),mapResolution,dilatedObstMap))&&...
          (norm(us)<0.0001*norm(u) || ((norm(us)<0.025*norm(u))&&(endDist<distThreshold)))

            disp(['SLQ found the optimal control input within ',num2str(iter-1),' iterations'])            
            
%             u = u + us;            
% %             x = xk + alfamin*xs;
%             % Forward integrate system dynamics
%             x = forwardIntegrateSystem(x,u);

            break;
        else
            if minimumMu >= minimumNu && size(p{mS},1) > 0
                I(p{mS}(iS(mS)),mS) = 0;
            elseif minimumMu < minimumNu && size(q{lS},1) > 0
                J(q{lS}(jS(lS)),lS) = 0;
            end
        end
    end
    
    if iter > maxIter
        cprintf('err','MMKP failed to generate a motion plan\n')
        if endDist > distThreshold
            error('The goal was not reachable');
        elseif norm(uh)>0.0001*norm(u)
            error('The SLQR algorithm failed to converge');
        else
            error('Something unexpected prevented SLQR to converge');
        end
    end
    
    disp(['Iteration number ',num2str(iter-1), ', dist to goal = ',num2str(endDist)])
    disp(['Is the path safe? ', num2str(isSafePath(x(10,:),x(11,:),mapResolution,dilatedObstMap))])    
    
end

toc

%% Plots
% Plotting stuff
map = [0 0.6   0
       0.6 0.3 0
       0.6 0   0];
colormap(map);
xVect = linspace(0,9.95,200);
[X,Y] = meshgrid(xVect,xVect);

x = forwardIntegrateSystem(x, u, dt);
    
for i = 2:size(t,2)
    if(x(16,i) < armJointsLimits(1,1) || x(16,i) > armJointsLimits(1,2))
        disp(['WARNING: Arm joint 1 is violating its position limits at waypoint ',num2str(i)]);
    end
    if(x(17,i) < armJointsLimits(2,1) || x(17,i) > armJointsLimits(2,2))
        disp(['WARNING: Arm joint 2 is violating its position limits at waypoint ',num2str(i)]);
    end
    if(x(18,i) < armJointsLimits(3,1) || x(18,i) > armJointsLimits(3,2))
        disp(['WARNING: Arm joint 3 is violating its position limits at waypoint ',num2str(i)]);
    end        
end

iu = cumsum(abs(x(25,:))*dt);
disp(['Total torque applied arm joint 1: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(26,:))*dt);
disp(['Total torque applied arm joint 2: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(27,:))*dt);
disp(['Total torque applied arm joint 3: ',num2str(iu(end)),' Nm'])    
iu = cumsum(abs(x(36,:))*dt);
disp(['Total torque applied wheel 1: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(37,:))*dt);
disp(['Total torque applied wheel 2: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(38,:))*dt);
disp(['Total torque applied wheel 3: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(39,:))*dt);
disp(['Total torque applied wheel 4: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(u(6,:)));
disp(['Total speed applied front steering joints: ',num2str(iu(end)),' rad/s'])
iu = cumsum(abs(u(7,:)));
disp(['Total speed applied back steering joints: ',num2str(iu(end)),' rad/s'])

figure(1)
hold off;
% Plotting first arm config
[TB0, TB1, TB2, TB3] = direct3(x(16:18,1));
TWB = getTraslation([x(10,1),x(11,1),zBC])*getZRot(x(12,1));
TW0 = TWB*TB0;
TW1 = TWB*TB1;
TW2 = TWB*TB2;
TW3 = TWB*TB3;
plot3([TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
      [TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
      [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)], 'Color', 'r', 'LineWidth', 2.5);
hold on;

% Plotting last arm config
[TB0, TB1, TB2, TB3] = direct3(x(16:18,end-1));
TWB = getTraslation([x(10,end-1),x(11,end-1),zBC])*getZRot(x(12,end-1));
TW0 = TWB*TB0;
TW1 = TWB*TB1;
TW2 = TWB*TB2;
TW3 = TWB*TB3;
plot3([TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
      [TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
      [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)], 'Color', 'r', 'LineWidth', 2.5);

% Plotting first rover position
TWB = getTraslation([x(10,1),x(11,1),zBC])*getZRot(x(12,1));
TB1 = getTraslation([dfy,dfx,-zBC]);
TB2 = getTraslation([-dfy,dfx,-zBC]);
TB3 = getTraslation([-dfy,-dfx,-zBC]);
TB4 = getTraslation([dfy,-dfx,-zBC]);
TW1 = TWB*TB1;
TW2 = TWB*TB2;
TW3 = TWB*TB3;
TW4 = TWB*TB4;
plot3([TWB(1,4) TW1(1,4) TWB(1,4) TW2(1,4) TWB(1,4) TW3(1,4) TWB(1,4) TW4(1,4)],...
      [TWB(2,4) TW1(2,4) TWB(2,4) TW2(2,4) TWB(2,4) TW3(2,4) TWB(2,4) TW4(2,4)],...
      [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', 'r', 'LineWidth', 2.5);
hold on;

quiver3(TWB(1,4), TWB(2,4), TWB(3,4), cos(x(12,1))/2, sin(x(12,1))/2, 0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.7)
quiver3(TWB(1,4), TWB(2,4), TWB(3,4), -sin(x(12,1))/2, cos(x(12,1))/2, 0, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.7)
quiver3(TWB(1,4), TWB(2,4), TWB(3,4), 0, 0, 1/2, 'Color', 'c', 'LineWidth', 2, 'MaxHeadSize', 0.7) 

% Plotting last rover position
TWB = getTraslation([x(10,end-1),x(11,end-1),zBC])*getZRot(x(12,end-1));
TW1 = TWB*TB1;
TW2 = TWB*TB2;
TW3 = TWB*TB3;
TW4 = TWB*TB4;
plot3([TWB(1,4) TW1(1,4) TWB(1,4) TW2(1,4) TWB(1,4) TW3(1,4) TWB(1,4) TW4(1,4)],...
          [TWB(2,4) TW1(2,4) TWB(2,4) TW2(2,4) TWB(2,4) TW3(2,4) TWB(2,4) TW4(2,4)],...
          [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', 'r', 'LineWidth', 2.5);

quiver3(TWB(1,4), TWB(2,4), TWB(3,4), cos(x(12,end-1))/2, sin(x(12,end-1))/2, 0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.7)
quiver3(TWB(1,4), TWB(2,4), TWB(3,4), -sin(x(12,end-1))/2, cos(x(12,end-1))/2, 0, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.7)
quiver3(TWB(1,4), TWB(2,4), TWB(3,4), 0, 0, 1/2, 'Color', 'c', 'LineWidth', 2, 'MaxHeadSize', 0.7)

% Plotting scenario
daspect([1 1 1])
contourf(X,Y,dilatedObstMap+obstMap);
plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 5, 'Color', 'y')
plot3(x(10,1:end-1),x(11,1:end-1),zBC*ones(size(x,2)-1), 'LineWidth', 5, 'Color', [1,0.5,0])
title('Mobile manipulator trajectories', 'interpreter', ...
'latex','fontsize',18)
plot3(x0(10,:),x0(11,:), zBC*ones(size(t,2),2), 'LineWidth', 5, 'Color', [0,0,0.6])
plot3(referencePath(1,:),referencePath(2,:), zBC*ones(size(t,2),2), 'LineWidth', 5, 'Color', [0,0,1])

plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', 'c')

hold off;


% figure(2)
% plot(t,x(16:18,:))
% title('Evolution of the arm joints', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\theta_1$','$\theta_2$','$\theta_3$', 'interpreter', ...
% 'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
% 
% figure(3)
% plot(t,u(1:3,:))
% title('Actuating joints speed','interpreter','latex')
% xlabel('t(s)','interpreter','latex','fontsize',18)
% ylabel('$\dot\theta(rad/s$)','interpreter','latex','fontsize',18)
% legend('$\dot\theta_1$','$\dot\theta_2$',...
% '$\dot\theta_3$','interpreter', ...
% 'latex','fontsize',18)
% 
% figure(4)
% plot(t,x(22:24,:))
% title('Evolution of the arm joints accelerations', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\ddot\theta_1$','$\ddot\theta_2$','$\ddot\theta_3$', 'interpreter', ...
% 'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\ddot\theta (rad/s^2)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
% figure(5)
% plot(t,x(25:27,:))
% title('Evolution of the applied arm torques', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\tau_1$','$\tau_2$','$\tau_3$', 'interpreter', ...
% 'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\tau (Nm)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
% figure(6)
% plot(t,x(28:31,:))
% title('Evolution of the applied wheel speeds', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\omega 1$','$\omega 2$','$\omega 3$',...
% '$\omega 4$', 'interpreter','latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\omega (rad/s)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
%     figure(7)
%     plot(t,x(32:35,:))
%     title('Evolution of the applied wheel accelerations', 'interpreter', ...
%     'latex','fontsize',18)
%     legend('$\dot\omega 1$','$\dot\omega 2$','$\dot\omega 3$',...
%     '$\dot\omega 4$', 'interpreter','latex','fontsize',18)
%     xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
%     ylabel('$\dot\omega (rad/s^2)$', 'interpreter', 'latex','fontsize',18)
%     grid
% 
%     figure(8)
%     plot(t,x(36:39,:))
%     hold on
%     yline(wheelTorqueLimit,'--');
%     yline(-wheelTorqueLimit,'--');
%     title('Evolution of the applied wheel torques', 'interpreter', ...
%     'latex','fontsize',18)
%     legend('$\tau_{\omega 1}$','$\tau_{\omega 2}$','$\tau_{\omega 3}$',...
%             '$\tau_{\omega 4}$', 'interpreter','latex','fontsize',18)
%     xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
%     ylabel('$\tau (Nm)$', 'interpreter', 'latex','fontsize',18)
%     grid
%     hold off
% 
%     figure(9)
%     plot(t,x(12,:))
%     title('Evolution of the vehicle heading', 'interpreter', ...
%     'latex','fontsize',18)
%     xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
%     ylabel('$\psi (rad)$', 'interpreter', 'latex','fontsize',18)
%     grid
% 
% figure(9)
% plot(t,x(12,:))
% title('Evolution of the vehicle heading', 'interpreter', ...
% 'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\psi (rad)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
% figure(10)
% plot(t,x(40:43,:))
% title('Evolution of the steering joints position', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\theta_1$','$\theta_2$','$\theta_3$',...
%         '$\theta_4$', 'interpreter','latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
% grid


%% Simulation

% sim('base_3DoF_dynamics_sim',t(end));



