%% Initialization

addpath(genpath('../../deps/ARES-DyMu_matlab'))
addpath(genpath('../../src'))
addpath('../../simscape')
addpath('../../maps')

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

[referencePath,~] = getPathGDM(totalCostMap,iInit,iGoal,tau, gTCMx, gTCMy);
referencePath = (referencePath-1)*mapResolution;

while(size(referencePath,1) > 1000)
    [referencePath,~] = getPathGDM(totalCostMap,iInit+round(2*rand(1,2)-1),iGoal,tau, gTCMx, gTCMy);
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


%% SLQR algorithm

% Plotting stuff
% map = [0 0.6   0
%        0.6 0.3 0
%        0.6 0   0];
% colormap(map);
% xVect = linspace(0,9.95,200);
% [X,Y] = meshgrid(xVect,xVect);

iter = 1;
while 1   
    % Forward integrate system equations
    x = forwardIntegrateSystem(x, u, dt);

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

            [pathi,~] = getPathGDM(totalCostMap,iInit,iGoal,tau, gTCMx, gTCMy);
            pathi = (pathi-1)*mapResolution;

            while(size(pathi,1) > 1000)
                [pathi,~] = getPathGDM(totalCostMap,iInit+round(2*rand(1,2)-1),iGoal,tau, gTCMx, gTCMy);
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
    
    % Update reference trajectories    
    xh0 = x0 - x;
    uh0 = u0 - u;    
    
    % Quadratize cost function along the trajectory
    Q = zeros(size(x,1),size(x,1),size(t,2));

    for i = 1:size(t,2)
%         [Tcmx, Tcmy] = getGradientTotalCost(x(10,i), x(11,i), mapResolution, gTCMx, gTCMy);
%         descYaw = atan2(-Tcmy, -Tcmx);
%         
%         headingDeviation = abs(x(12,i) - descYaw);
%         while headingDeviation > 2*pi
%             headingDeviation = abs(headingDeviation - 2*pi);
%         end
%         
%         % Quadratic cost in function of the heading, greater when similar
%         % to the descient gradient direction of the FMM total cost map
%         Q(10:12,10:12,i) = (rtc - rtc*rtor/pi*(3/rtor-4)*headingDeviation + 2*rtc*rtor*(1/rtor-2)/pi^2 * headingDeviation^2)*eye(3,3);
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
    
    Qend = zeros(size(x,1),size(x,1));
    
    Qend(1,1) = fc;
    Qend(2,2) = fc;
    Qend(3,3) = fc;
    Qend(7,7) = foc;
    Qend(8,8) = foc;
    Qend(9,9) = foc;
    Qend(13,13) = fsc;
    Qend(14,14) = fsc;
    Qend(15,15) = fsc;
    
    R = eye(size(u,1));
    R(1,1) = ac1;
    R(2,2) = ac2;
    R(3,3) = ac3;
    R(4,4) = bc;
    R(5,5) = bc;  
    R(6,6) = sc;
    R(7,7) = sc; 
    
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
    
    % Obstacles limits cost
    Ox = zeros(size(Q,1),size(t,2));
    for i = 1:size(t,2)-1
        [Ox(10,i), Ox(11,i)] = getGradientTotalCost(x(10,i), x(11,i), mapResolution, gOMx, gOMy);
        Ox(10,i) = oc*Ox(10,i);
        Ox(11,i) = oc*Ox(11,i);
    end
    
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
    
    P(:,:,end) = Qend;
    s(:,:,end) = -Qend*xh0(:,end) + Ox(:,end);
    
    xh = zeros(size(x,1),size(t,2));
    uh = zeros(size(u,1),size(t,2));
    v = zeros(size(xh));
    lambdah = zeros(size(s));
        
    % Solve backward
    for i = size(t,2)-1:-1:1
        M(:,:,i) = inv(eye(size(B,1)) + B(:,:,i)/R*B(:,:,i).'*P(:,:,i+1));
        P(:,:,i) = Q(:,:,i) + A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*A(:,:,i);
        s(:,:,i) = A(:,:,i).'*(eye(size(Q,1)) - P(:,:,i+1)*M(:,:,i)*B(:,:,i)/R*B(:,:,i).')*s(:,:,i+1)+...
            A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*B(:,:,i)*uh0(:,i) - Q(:,:,i)*xh0(:,i)...
            + Ox(:,i);
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
    if (isSafePath(x(10,:),x(11,:),mapResolution,dilatedObstMap))&&...
        (norm(uh)<0.0001*norm(u) || ((norm(uh)<0.2*norm(u))&&(endDist<distThreshold)))
        disp(['SLQ found the optimal control input within ',num2str(iter-1),' iterations'])
        break;
    else
        % Line search to optimize alfa
        alfa = 1:-lineSearchStep:0.0001;
        J = zeros(size(alfa));
        uk = u;
        for n = 1:size(alfa,2)
            u = uk + alfa(n)*uh;
            x = forwardIntegrateSystem(x, u, dt);

            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Qend*(x(:,end)-x0(:,end))...
                + 100*~isSafePath(x(1,:),x(2,:),mapResolution,dilatedObstMap)...
                + tc*getTotalCost(x(10,end), x(11,end), mapResolution, totalCostMap);
            for i = 1:size(t,2)-1
                J(n) = J(n) + 1/2*((x(:,i)-x0(:,i)).'*Q(:,:,i)*(x(:,i)-x0(:,i))...
                    + (u(:,i)-u0(:,i)).'*R*(u(:,i)-u0(:,i)))...
                    + tc*getTotalCost(x(10,i), x(11,i), mapResolution, totalCostMap);
            end            
        end
        [mincost, ind] = min(J);
        alfamin = alfa(ind);
        
        % Update controller
        u = uk + alfamin*uh;
    end
    
    
    if iter > maxIter
        cprintf('err','MMKP failed to generate a motion plan\n')
        if ~isSafePath(x(10,:),x(11,:),mapResolution,dilatedObstMap)
            error('The generated path is not safe');
        elseif endDist > distThreshold
            error('The goal was not reachable');
        elseif norm(uh)>0.0001*norm(u)
            error('The SLQR algorithm failed to converge');
        else
            error('Something unexpected prevented SLQR to converge');
        end
    end
    
%     figure(1)
%     % Plotting first arm config
%     [TB0, TB1, TB2, TB3] = direct3(x(16:18,1));
%     TWB = getTraslation([x(10,1),x(11,1),zBC])*getZRot(x(12,1));
%     TW0 = TWB*TB0;
%     TW1 = TWB*TB1;
%     TW2 = TWB*TB2;
%     TW3 = TWB*TB3;
%     plot3([TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
%           [TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
%           [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)], 'Color', 'r', 'LineWidth', 2.5);
%     hold on;
%     
%     % Plotting last arm config
%     [TB0, TB1, TB2, TB3] = direct3(x(16:18,end-1));
%     TWB = getTraslation([x(10,end-1),x(11,end-1),zBC])*getZRot(x(12,end-1));
%     TW0 = TWB*TB0;
%     TW1 = TWB*TB1;
%     TW2 = TWB*TB2;
%     TW3 = TWB*TB3;
%     plot3([TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4)],...
%           [TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4)],...
%           [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)], 'Color', 'r', 'LineWidth', 2.5);
%       
%     % Plotting first rover position
%     TWB = getTraslation([x(10,1),x(11,1),zBC])*getZRot(x(12,1));
%     TB1 = getTraslation([dfy,dfx,-zBC]);
%     TB2 = getTraslation([-dfy,dfx,-zBC]);
%     TB3 = getTraslation([-dfy,-dfx,-zBC]);
%     TB4 = getTraslation([dfy,-dfx,-zBC]);
%     TW1 = TWB*TB1;
%     TW2 = TWB*TB2;
%     TW3 = TWB*TB3;
%     TW4 = TWB*TB4;
%     plot3([TWB(1,4) TW1(1,4) TWB(1,4) TW2(1,4) TWB(1,4) TW3(1,4) TWB(1,4) TW4(1,4)],...
%           [TWB(2,4) TW1(2,4) TWB(2,4) TW2(2,4) TWB(2,4) TW3(2,4) TWB(2,4) TW4(2,4)],...
%           [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', 'r', 'LineWidth', 2.5);
%     hold on;
%    
%     quiver3(TWB(1,4), TWB(2,4), TWB(3,4), cos(x(12,1))/2, sin(x(12,1))/2, 0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.7)
%     quiver3(TWB(1,4), TWB(2,4), TWB(3,4), -sin(x(12,1))/2, cos(x(12,1))/2, 0, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.7)
%     quiver3(TWB(1,4), TWB(2,4), TWB(3,4), 0, 0, 1/2, 'Color', 'c', 'LineWidth', 2, 'MaxHeadSize', 0.7) 
%     
%     % Plotting last rover position
%     TWB = getTraslation([x(10,end-1),x(11,end-1),zBC])*getZRot(x(12,end-1));
%     TW1 = TWB*TB1;
%     TW2 = TWB*TB2;
%     TW3 = TWB*TB3;
%     TW4 = TWB*TB4;
%     plot3([TWB(1,4) TW1(1,4) TWB(1,4) TW2(1,4) TWB(1,4) TW3(1,4) TWB(1,4) TW4(1,4)],...
%               [TWB(2,4) TW1(2,4) TWB(2,4) TW2(2,4) TWB(2,4) TW3(2,4) TWB(2,4) TW4(2,4)],...
%               [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', 'r', 'LineWidth', 2.5);
% 
%     quiver3(TWB(1,4), TWB(2,4), TWB(3,4), cos(x(12,end-1))/2, sin(x(12,end-1))/2, 0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.7)
%     quiver3(TWB(1,4), TWB(2,4), TWB(3,4), -sin(x(12,end-1))/2, cos(x(12,end-1))/2, 0, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.7)
%     quiver3(TWB(1,4), TWB(2,4), TWB(3,4), 0, 0, 1/2, 'Color', 'c', 'LineWidth', 2, 'MaxHeadSize', 0.7)
%       
%     % Plotting scenario
%     daspect([1 1 1])
%     contourf(X,Y,dilatedObstMap+obstMap);
%     plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 5, 'Color', 'y')
%     plot3(x(10,1:end-1),x(11,1:end-1),zBC*ones(size(x,2)-1), 'LineWidth', 5, 'Color', [1,0.5,0])
%     title('Mobile manipulator trajectories', 'interpreter', ...
%     'latex','fontsize',18)
%     plot3(x0(10,:),x0(11,:), zBC*ones(size(t,2),2), 'LineWidth', 5, 'Color', [0,0,0.6])
%     plot3(referencePath(1,:),referencePath(2,:), zBC*ones(size(t,2),2), 'LineWidth', 5, 'Color', [0,0,1])
%     plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', 'c')
% 
%     hold off;
    
    disp(['Iteration number ',num2str(iter-1), ', alpha = ', num2str(alfamin), ', endDist = ',num2str(endDist)])
    disp(['Is the path safe? ', num2str(isSafePath(x(10,:),x(11,:),mapResolution,dilatedObstMap))])
end

%% Plots
% Plotting stuff
map = [0 0.6   0
       0.6 0.3 0
       0.6 0   0];
colormap(map);
xVect = linspace(0,9.95,200);
[X,Y] = meshgrid(xVect,xVect);

x = forwardIntegrateSystem(x, u, dt);

toc
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
% iu = cumsum(abs(u(6,:)));
% disp(['Total speed applied front steering joints: ',num2str(iu(end)),' rad/s'])
% iu = cumsum(abs(u(7,:)));
% disp(['Total speed applied back steering joints: ',num2str(iu(end)),' rad/s'])

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
      [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)], 'Color', [0.5 0.5 0.5], 'LineWidth', 2.5);
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
      [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4)], 'Color', [0.5 0.5 0.5], 'LineWidth', 2.5);

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
%        'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
% grid


figure(3)
plot(t,u(1:3,:))
title('Actuating joints speed','interpreter','latex')
xlabel('t(s)','interpreter','latex','fontsize',18)
ylabel('$\dot\theta(rad/s$)','interpreter','latex','fontsize',18)
legend('$\dot\theta_1$','$\dot\theta_2$',...
       '$\dot\theta_3$','interpreter', ...
       'latex','fontsize',18)

% figure(4)
% plot(t,x(22:24,:))
% title('Evolution of the arm joints accelerations', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\ddot\theta_1$','$\ddot\theta_2$','$\ddot\theta_3$', 'interpreter', ...
%        'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\ddot\theta (rad/s^2)$', 'interpreter', 'latex','fontsize',18)
% grid

figure(5)
plot(t,x(25:27,:))
title('Evolution of the applied arm torques', 'interpreter', ...
'latex','fontsize',18)
legend('$\tau_1$','$\tau_2$','$\tau_3$', 'interpreter', ...
       'latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
ylabel('$\tau (Nm)$', 'interpreter', 'latex','fontsize',18)
grid

% figure(6)
% plot(t,x(28:31,:))
% title('Evolution of the applied wheel speeds', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\omega 1$','$\omega 2$','$\omega 3$',...
%         '$\omega 4$', 'interpreter','latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\omega (rad/s)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
% figure(7)
% plot(t,x(32:35,:))
% title('Evolution of the applied wheel accelerations', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\dot\omega 1$','$\dot\omega 2$','$\dot\omega 3$',...
%         '$\dot\omega 4$', 'interpreter','latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\dot\omega (rad/s^2)$', 'interpreter', 'latex','fontsize',18)
% grid

figure(8)
plot(t,x(36:39,:))
hold on
yline(wheelTorqueLimit,'--r','Max torque');
yline(-wheelTorqueLimit,'--r','Min torque');
title('Evolution of the applied wheel torques', 'interpreter', ...
'latex','fontsize',18)
legend('$\tau_{\omega 1}$','$\tau_{\omega 2}$','$\tau_{\omega 3}$',...
        '$\tau_{\omega 4}$', 'interpreter','latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
ylabel('$\tau (Nm)$', 'interpreter', 'latex','fontsize',18)
grid
hold off

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

speed = zeros(size(t));
for i = 1:size(t,2)
    speedSign = sign(sum(x(28:31,i)));
    speed(i) = speedSign*sqrt(x(13,i)^2+x(14,i)^2);
end
averageSpeed = sum(speed)/size(speed,2);

figure(11)
plot(t,speed)
hold on;
yline(averageSpeed,'--r','Average speed');
yline(vehicleSpeed,'--b','Commanded speed');
title('Evolution of the vehicle speed', 'interpreter', ...
'latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
ylabel('$v (m/s)$', 'interpreter', 'latex','fontsize',18)
grid
hold off;


%% Simulation

% sim('base_3DoF_dynamics_sim',t(end));

