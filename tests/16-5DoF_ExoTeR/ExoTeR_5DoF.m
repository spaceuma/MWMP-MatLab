%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MP-FB %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                Motion Planning of the Full robot Body                   %
%                         University of Malaga                            %
%                    Author: Gonzalo Jesús Paz Delgado                    %
%                      E-mail: gonzalopd96@uma.es                         %
%              Supervisor: Carlos J. Pérez del Pulgar (UMA)               %
%                                                                         %
%       ExoTeR_5DoF: Script to simulate a sample fetching operation       %
%                    with a 5DoF manipulator and differential steering    %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization

addpath(genpath('../../deps/ARES-DyMu_matlab'))
addpath(genpath('../../src'))
addpath(genpath('../../simscape'))
addpath('../../maps')

clear

tic

% System properties
global d1 a2 a3 d5 d6;
d1 = 0.0895;
a2 = 0.206;
a3 = 0.176;
d5 = 0.0555;
d6 = 0.14;

global width1 width2 width3 width4 width5;
width1 = 0.055;
width2 = 0.02;
width3 = 0.02;
width4 = 0.05;
width5 = 0.02;

global dfx dfy;
dfx = 0.3;
dfy = 0.27;

global zGC xCB yCB zCB;
zGC = 0.202;
xCB = 0.165;
yCB = -0.15;
zCB = 0.028;

global reachabilityDistance;
reachabilityDistance = d1+a2+a3+d5+d6-zGC-zCB+xCB;

global armHeight armWidth;
armHeight = 0.05;
armWidth = 0.1;

global wheelRadius wheelWidth wheelMass;
wheelRadius = 0.07;
wheelWidth = 0.09;
wheelMass = 0.484;

global vehicleMass;
vehicleMass = 15;

global m1 m2 m3 m4 m5;
m1 = 0.5;
m2 = 0.8;
m3 = 0.8;
m4 = 0.3;
m5 = 0.2;

global rollingResistance;
rollingResistance = 0.0036;

global g;
g = 9.81;
               
%% Initial state and goal
% Initial base pose
xC0 = 2.00;
yC0 = 2.80;
zC0 = zGC;
yawC0 = -pi/6;

% Initial configuration
qi = [0.5708, -pi, +2.21, pi/2, 0];

% Initial ee pose
TWC = getTraslation([xC0,yC0,zC0])*getZRot(yawC0);
[~, ~, ~, ~, ~,TB5] = direct5(qi);

TCB = getTraslation([xCB, yCB, zCB])*getYRot(pi/2);

TW5 = TWC*TCB*TB5;
TC5 = TCB*TB5;

xei = TW5(1,4);
yei = TW5(2,4);
zei = TW5(3,4);
rollei = 0;
pitchei = 2.21;
yawei = 0.5708;

% Goal end effector pose
xef = 2.6;
yef = 2.80;
zef = 0.10;
rollef = -pi/2;
pitchef = pi;
yawef = 0;

%% Configuration variables

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% General configuration %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Name of the obst map to be used
obstMapFile = 'obstMap4';

% Number of timesteps
timeSteps = 500;

% Maximum number of iterations
maxIter = 500;

% Activate/deactivate stepped procedure for checking constraints:
% - 0 to use a not stepped, not constrained procedure using simple SLQ only
% - 1 to activate the stepped procedure
% - 2 to use a not stepped procedure using constrained SLQ only
stepped = 1;

% Activate/deactivate dynamic plotting during the simulation
dynamicPlotting = false;

% Vehicle goal average speed (m/s)
vehicleSpeed = 0.1;

% Percentage of the base path size (%) where to start reducing
% progressively the vehicles speed until completely stopped
startBaseSpeedReduction = 90;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% FMM configuration %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Distance to obstacles considered risky (should be an obstacle)
riskDistance = 0.30;

% Distance to obstacles considered enough for safety (should have bigger
% cost to traverse areas with less than this distance to obstacles)
safetyDistance = 1.00;

% Gradient Descent Method step size, as a percentage of the map resolution
tau = 0.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% SLQ algorithm configuration %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Minimum step actuation percentage
config.lineSearchStep = 0.32; 

% Percentage of constrained timesteps for resampling
config.resamplingThreshold = 30;

% Percentage of step actuation to consider convergence
config.controlThreshold = 5e-2;

% Check distance to goal for convergence
config.checkDistance = true;
if config.checkDistance
    config.distIndexes = [1 2 3];
end

% Max acceptable dist (m)
config.distThreshold = 0.01;

% Check final orientation for convergence
config.checkOrientation = true;
if config.checkOrientation
    config.orientationIndexes = [16 8 20];
end

% Max acceptable final orientation euclidean distance
config.orientationThreshold = 0.05;

% Check constraints compliance for convergence, only with constrained SLQ
config.checkConstraints = true;

% Check safety of the state for convergence
config.checkSafety = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Reference path updating configuration %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Activate/deactivate dynamic reference path updating
updateReference = false;

% Percentage of path wayp to be checked when generating new references
config.percentageMCM = 80;

% Max distance to the reference to update it
config.maxDistUpdate = 2.0;

% Min distance to the reference to update it
config.minDistUpdate = 0.5;

% Percentage of cost reduction to update the reference path
config.costThreshold = 5;

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% LQR costs %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%

% State costs
fci = 1000000000; % Final state cost, 1000000000
foci = 1000000000; % Final orientation cost, 1000000000
fsci = 1000000000; % Final zero speed cost, 1000000000
rtci = 20; % Reference path max cost, 20
oci = 200.0; % Obstacles repulsive cost, 70.0

tau1ci = 2.0; % Joint 1 inverse torque constant, 2
tau2ci = 2.0; % Joint 2 inverse torque constant, 2
tau3ci = 2.0; % Joint 3 inverse torque constant, 2
tau4ci = 2.0; % Joint 4 inverse torque constant, 2
tau5ci = 2.0; % Joint 5 inverse torque constant, 2

tauWheeli = 15000.0; % Wheels joint inverse torque constant, 640

% Input costs
ac1i = 1000000; % Arm actuation cost, 100000000
ac2i = 1000000; % Arm actuation cost, 100000000
ac3i = 1000000; % Arm actuation cost, 100000000
ac4i = 1000000; % Arm actuation cost, 100000000
ac5i = 1000000; % Arm actuation cost, 100000000
bci = 1000; % Base actuation cost, 9

% Extra costs
kappa1 = 0.02; % Influence of yaw into rover pose, tune till convergence

%% Reference trajectory computation
% FMM to compute totalCostMap
% Loading obstacles map
load(obstMapFile,'obstMap')
mapResolution = 0.05;

% Initial spot and goal indexes in the map
iInit = [round(xC0/mapResolution)+1 round(yC0/mapResolution)+1];
iGoal = [round(xef/mapResolution)+1 round(yef/mapResolution)+1];

% Generating a fake obstacle on the sample to avoid stepping on it
obstMap(iGoal(2), iGoal(1)) = 1;

% Dilating obstacles map to ensure rover safety
dilatedObstMap = dilateObstMap(obstMap, riskDistance, mapResolution);
safeObstMap = dilateObstMap(obstMap, safetyDistance, mapResolution);

% Generating cost map for FMM
costMap = ones(size(obstMap));
distMap = mapResolution*bwdist(obstMap);
auxMap = 1./distMap;
gradient = 1;
minCost = max(max(costMap(costMap~=Inf)));
costMap(safeObstMap==1)= minCost + gradient*auxMap(safeObstMap==1)./min(min(auxMap(safeObstMap==1)));
costMap(safeObstMap==1)= costMap(safeObstMap==1)./min(min(costMap(safeObstMap==1)));
costMap(obstMap==1) = max(max(costMap(costMap~=Inf)));

% Computing total cost map
[totalCostMap, ~] = computeTmap(costMap,iGoal);

% Computing gradient
totalCostMap(totalCostMap == Inf) = NaN;
[gTCMx, gTCMy] = calculateGradient(mapResolution*totalCostMap);

% Extracting the reference path
[referencePath,~] = getPathGDM(totalCostMap,iInit,iGoal,tau, gTCMx, gTCMy);

% Ensuring no singularities happened in the gradient
while(size(referencePath,1) > 1000)
    [referencePath,~] = getPathGDM(totalCostMap,iInit+round(2*rand(1,2)-1),iGoal,tau, gTCMx, gTCMy);
end

% Translating from indexes to meters
referencePath = (referencePath-1)*mapResolution;

%% Time horizon estimation
pathLength = getLength(referencePath.');
expectedTimeArrival = pathLength/vehicleSpeed;
if pathLength > 10
    warning(['The objective is too far for an efficient motion plan ',...
            'computation, the results may be unaccurate']);
elseif pathLength < reachabilityDistance
    warning(['The objective is already close to the rover, ',...
             'the expected time horizon will be set to 60 seconds']);
    expectedTimeArrival = 160;
end

% Time vector
tf = expectedTimeArrival; 
dt = tf/(timeSteps-1);
t = 0:dt:tf;

%% Initial reference path adaptation to the state space model
% Resizing path
x1 = 1:size(referencePath,1);
x2 = linspace(1,size(referencePath,1),timeSteps);
referencePath = interp1(x1,referencePath,x2);

% Computing path yaw
yaw = getYaw(referencePath);
referencePath = [referencePath yaw].';

waypSeparation = norm(referencePath(1:2,1)-referencePath(1:2,2));

% Ensure the yaw is continuous and has no jumps between -pi and pi
yawC0 = modulateYaw(yawC0,referencePath(3,1));
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

%% Definitive Costs
time_ratio = tf/60; % Ratio for the costs to ensure convergence

% State costs
fc = fci/time_ratio; % Final state cost
foc = foci/time_ratio; % Final orientation cost
fsc = fsci/time_ratio; % Final zero speed cost
rtc = rtci*time_ratio; % Reference path max cost
oc = oci*time_ratio; % Obstacles repulsive cost

tau1c = tau1ci/time_ratio; % Joint 1 inverse torque constant
tau2c = tau2ci/time_ratio; % Joint 2 inverse torque constant
tau3c = tau3ci/time_ratio; % Joint 3 inverse torque constant
tau4c = tau4ci/time_ratio; % Joint 3 inverse torque constant
tau5c = tau5ci/time_ratio; % Joint 3 inverse torque constant

tauWheel = tauWheeli/time_ratio; % Wheels joint inverse torque constant

% Input costs
bc = bci/time_ratio; % Base actuation cost
ac1 = ac1i*time_ratio; % Arm actuation cost
ac2 = ac2i*time_ratio; % Arm actuation cost
ac3 = ac3i*time_ratio; % Arm actuation cost
ac4 = ac4i*time_ratio; % Arm actuation cost
ac5 = ac5i*time_ratio; % Arm actuation cost

%% Generate map info
map.mapResolution = mapResolution;
map.obstMap = dilatedObstMap;
map.gradientObstaclesMapX = gOMx;
map.gradientObstaclesMapY = gOMy;
map.obstaclesCost = oc;
map.totalCostMap = totalCostMap;
map.gradientTotalCostMapX = gTCMx;
map.gradientTotalCostMapY = gTCMy;
map.iGoal = iGoal;

%% Generate state space model info
stateSpaceModel.XYIndexes = [10 11];
stateSpaceModel.thetaIndex = 12;

%% Generate trajectory info
trajectoryInfo.tau = tau;
trajectoryInfo.waypointSeparation = waypSeparation;

%% State space model
% State vectors
numStates = 41;
x = zeros(numStates,timeSteps);
% WTEE
x(1,1) = xei;
x(2,1) = yei;
x(3,1) = zei;
% BTEE
x(4,1) = -TC5(3,4);
x(5,1) = TC5(2,4);
x(6,1) = TC5(1,4);
x(7,1) = rollei;
x(8,1) = pitchei;
x(9,1) = yawei;
% WTC
x(10,1) = xC0;
x(11,1) = yC0;
x(12,1) = yawC0;
% Bspeed
x(13,1) = 0;
x(14,1) = 0;
x(15,1) = 0;
% Arm joints positions
x(16,1) = qi(1);
x(17,1) = qi(2);
x(18,1) = qi(3);
x(19,1) = qi(4);
x(20,1) = qi(5);
% Arm joints speeds
x(21,1) = 0;
x(22,1) = 0;
x(23,1) = 0;
x(24,1) = 0;
x(25,1) = 0;
% Arm joints accelerations
x(26,1) = 0;
x(27,1) = 0;
x(28,1) = 0;
x(29,1) = 0;
x(30,1) = 0;
% Arm joints torques
x(31,1) = 0;
x(32,1) = 0;
x(33,1) = 0;
x(34,1) = 0;
x(35,1) = 0;
% Wheels speed
x(36,1) = 0;
x(37,1) = 0;
% Wheels acceleration
x(38,1) = 0;
x(39,1) = 0;
% Wheels torque
x(40,1) = 0;
x(41,1) = 0;

% Initial control law
numInputs = 7;
u = zeros(numInputs,timeSteps);

% Target state and control trajectories
x0 = zeros(numStates,timeSteps);

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
% WTC
% x0(10,end) = 0;
% x0(11,end) = 0;
% x0(12,end) = 0;
% Bspeed
x0(13,end) = 0;
x0(14,end) = 0;
x0(15,end) = 0;
% Arm joints positions
x0(16,end) = yawef;
x0(17,end) = 0;
x0(18,end) = 0;
x0(19,end) = 0;
x0(20,end) = rollef;
% Arm joints speeds
x0(21,end) = 0;
x0(22,end) = 0;
x0(23,end) = 0;
x0(24,end) = 0;
x0(25,end) = 0;
% Arm joints accelerations
x0(26,end) = 0;
x0(27,end) = 0;
x0(28,end) = 0;
x0(29,end) = 0;
x0(30,end) = 0;
% Arm joints torques
x0(31,end) = 0;
x0(32,end) = 0;
x0(33,end) = 0;
x0(34,end) = 0;
x0(35,end) = 0;
% Wheels speed
x0(36,end) = 0;
x0(37,end) = 0;
% Wheels acceleration
x0(38,end) = 0;
x0(39,end) = 0;
% Wheels torque
x0(40,end) = 0;
x0(41,end) = 0;

u0 = zeros(numInputs,timeSteps);

% Forward integrate system equations
x = forwardIntegrateSystem(x, u, dt);

xini = x;
uini = u;

%% Constraints matrices definition
% State input constraints
numStateInputConstraints = 10;
definedConstraints = 0;
I0 = zeros(numStateInputConstraints,timeSteps);
I = I0;
C = zeros(numStateInputConstraints,numStates,timeSteps);
D = zeros(numStateInputConstraints,numInputs,timeSteps);
r = zeros(numStateInputConstraints,timeSteps);

% The state input constraints are defined as:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% C*x + D*u + r <= 0 %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Arm joints speed limit
armJointsSpeedLimit = 0.01; % rad/s

D(definedConstraints+1,1,:) = -1;
r(definedConstraints+1,:) = -armJointsSpeedLimit;
definedConstraints = definedConstraints+1;

D(definedConstraints+1,1,:) = 1;
r(definedConstraints+1,:) = -armJointsSpeedLimit;
definedConstraints = definedConstraints+1;

D(definedConstraints+1,2,:) = -1;
r(definedConstraints+1,:) = -armJointsSpeedLimit;
definedConstraints = definedConstraints+1;

D(definedConstraints+1,2,:) = 1;
r(definedConstraints+1,:) = -armJointsSpeedLimit;
definedConstraints = definedConstraints+1;

D(definedConstraints+1,3,:) = -1;
r(definedConstraints+1,:) = -armJointsSpeedLimit;
definedConstraints = definedConstraints+1;

D(definedConstraints+1,3,:) = 1;
r(definedConstraints+1,:) = -armJointsSpeedLimit;
definedConstraints = definedConstraints+1;

D(definedConstraints+1,4,:) = -1;
r(definedConstraints+1,:) = -armJointsSpeedLimit;
definedConstraints = definedConstraints+1;

D(definedConstraints+1,4,:) = 1;
r(definedConstraints+1,:) = -armJointsSpeedLimit;
definedConstraints = definedConstraints+1;

D(definedConstraints+1,5,:) = -1;
r(definedConstraints+1,:) = -armJointsSpeedLimit;
definedConstraints = definedConstraints+1;

D(definedConstraints+1,5,:) = 1;
r(definedConstraints+1,:) = -armJointsSpeedLimit;
definedConstraints = definedConstraints+1;


if definedConstraints ~= numStateInputConstraints
    error(['The specified number of state input constraints does not'...
           ' match the size of the defined matrices D and r.']);
end

% Pure state constraints
numPureStateConstraints = 14;
definedConstraints = 0;
J0 = zeros(numPureStateConstraints,timeSteps);
J = J0;
G = zeros(numPureStateConstraints,numStates,timeSteps);
h = zeros(numPureStateConstraints,timeSteps);

% The pure state constraints are defined as:
%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% G*x + h <= 0 %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%

% Arm position constraints
armJointsPositionLimits = [-30 +90;
                   -250 +70;
                   -160 +160;
                   -70 +250;
                   -160 +160]*pi/180; % rad
               
G(definedConstraints+1,16,:) = -1;
h(definedConstraints+1,:) = armJointsPositionLimits(1,1);
definedConstraints = definedConstraints+1;

G(definedConstraints+1,16,:) = 1;
h(definedConstraints+1,:) = -armJointsPositionLimits(1,2);
definedConstraints = definedConstraints+1;

G(definedConstraints+1,17,:) = -1;
h(definedConstraints+1,:) = armJointsPositionLimits(2,1);
definedConstraints = definedConstraints+1;

G(definedConstraints+1,17,:) = 1;
h(definedConstraints+1,:) = -armJointsPositionLimits(2,2);
definedConstraints = definedConstraints+1;

G(definedConstraints+1,18,:) = -1;
h(definedConstraints+1,:) = armJointsPositionLimits(3,1);
definedConstraints = definedConstraints+1;

G(definedConstraints+1,18,:) = 1;
h(definedConstraints+1,:) = -armJointsPositionLimits(3,2);
definedConstraints = definedConstraints+1;

G(definedConstraints+1,19,:) = -1;
h(definedConstraints+1,:) = armJointsPositionLimits(4,1);
definedConstraints = definedConstraints+1;

G(definedConstraints+1,19,:) = 1;
h(definedConstraints+1,:) = -armJointsPositionLimits(4,2);
definedConstraints = definedConstraints+1;

G(definedConstraints+1,20,:) = -1;
h(definedConstraints+1,:) = armJointsPositionLimits(5,1);
definedConstraints = definedConstraints+1;

G(definedConstraints+1,20,:) = 1;
h(definedConstraints+1,:) = -armJointsPositionLimits(5,2);
definedConstraints = definedConstraints+1;

% Wheel torque constraints
wheelTorqueLimit = 2.85;

G(definedConstraints+1,40,:) = 1;
h(definedConstraints+1,:) = -wheelTorqueLimit;
definedConstraints = definedConstraints+1;

G(definedConstraints+1,41,:) = 1;
h(definedConstraints+1,:) = -wheelTorqueLimit;
definedConstraints = definedConstraints+1;

G(definedConstraints+1,40,:) = -1;
h(definedConstraints+1,:) = -wheelTorqueLimit;
definedConstraints = definedConstraints+1;

G(definedConstraints+1,41,:) = -1;
h(definedConstraints+1,:) = -wheelTorqueLimit;
definedConstraints = definedConstraints+1;

if definedConstraints ~= numPureStateConstraints
    error(['The specified number of pure state constraints does not'...
           ' match the size of the defined matrices G and h.']);
end

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
cmap = [0 0.6   0
       0.6 0.3 0
       0.6 0   0];
colormap(cmap);
xVect = linspace(0,(size(obstMap,1)-1)*mapResolution,size(obstMap,1));
[X,Y] = meshgrid(xVect,xVect);

figure(1)
clf(1)
% Plotting first arm config
[TB0, TB1, TB2, TB3, TB4, TB5] = realDirect5(x(16:20,1));
TWC = getTraslation([x(10,1),x(11,1),zGC])*getZRot(x(12,1));
TW0 = TWC*TCB*TB0;
TWWh1 = TWC*TCB*TB1;
TWWh2 = TWC*TCB*TB2;
TWWh3 = TWC*TCB*TB3;
TWWh4 = TWC*TCB*TB4;
TW5 = TWC*TCB*TB5;
h1 = plot3([TW0(1,4) TWWh1(1,4) TWWh2(1,4) TWWh3(1,4) TWWh4(1,4) TW5(1,4)],...
           [TW0(2,4) TWWh1(2,4) TWWh2(2,4) TWWh3(2,4) TWWh4(2,4) TW5(2,4)],...
           [TW0(3,4) TWWh1(3,4) TWWh2(3,4) TWWh3(3,4) TWWh4(3,4) TW5(3,4)],...
                         'Color', [0.8 0.8 0.8], 'LineWidth', 2.5);
hold on;
    
% Plotting first rover position [TODO] Update the representation to look
% like exoter
TWC = getTraslation([x(10,1),x(11,1),zGC])*getZRot(x(12,1));
TCWh1 = getTraslation([dfy,dfx,-zGC]);
TCWh2 = getTraslation([-dfy,dfx,-zGC]);
TCWh3 = getTraslation([-dfy,-dfx,-zGC]);
TCWh4 = getTraslation([dfy,-dfx,-zGC]);
TWWh1 = TWC*TCWh1;
TWWh2 = TWC*TCWh2;
TWWh3 = TWC*TCWh3;
TWWh4 = TWC*TCWh4;
h2 = plot3([TWC(1,4) TWWh1(1,4) TWC(1,4) TWWh2(1,4) TWC(1,4) TWWh3(1,4) TWC(1,4) TWWh4(1,4)],...
      [TWC(2,4) TWWh1(2,4) TWC(2,4) TWWh2(2,4) TWC(2,4) TWWh3(2,4) TWC(2,4) TWWh4(2,4)],...
      [TWC(3,4) TWWh1(3,4) TWC(3,4) TWWh2(3,4) TWC(3,4) TWWh3(3,4) TWC(3,4) TWWh4(3,4)], 'Color', [0.4 0.4 0.4], 'LineWidth', 2.5);

% Plotting the first base frame
h3 = plotFrame(TWC, 0.3);

% Plotting last arm config
[TB0, TB1, TB2, TB3, TB4, TB5] = realDirect5(x(16:20,end-1));
TWC = getTraslation([x(10,end-1),x(11,end-1),zGC])*getZRot(x(12,end-1));
TW0 = TWC*TCB*TB0;
TWWh1 = TWC*TCB*TB1;
TWWh2 = TWC*TCB*TB2;
TWWh3 = TWC*TCB*TB3;
TWWh4 = TWC*TCB*TB4;
TW5 = TWC*TCB*TB5;
h4 = plot3([TW0(1,4) TWWh1(1,4) TWWh2(1,4) TWWh3(1,4) TWWh4(1,4) TW5(1,4)],...
           [TW0(2,4) TWWh1(2,4) TWWh2(2,4) TWWh3(2,4) TWWh4(2,4) TW5(2,4)],...
           [TW0(3,4) TWWh1(3,4) TWWh2(3,4) TWWh3(3,4) TWWh4(3,4) TW5(3,4)],...
                         'Color', [0.8 0.8 0.8], 'LineWidth', 2.5);
                     
% Plotting last rover position
TWC = getTraslation([x(10,end-1),x(11,end-1),zGC])*getZRot(x(12,end-1));
TCWh1 = getTraslation([dfy,dfx,-zGC]);
TCWh2 = getTraslation([-dfy,dfx,-zGC]);
TCWh3 = getTraslation([-dfy,-dfx,-zGC]);
TCWh4 = getTraslation([dfy,-dfx,-zGC]);
TWWh1 = TWC*TCWh1;
TWWh2 = TWC*TCWh2;
TWWh3 = TWC*TCWh3;
TWWh4 = TWC*TCWh4;
h5 = plot3([TWC(1,4) TWWh1(1,4) TWC(1,4) TWWh2(1,4) TWC(1,4) TWWh3(1,4) TWC(1,4) TWWh4(1,4)],...
          [TWC(2,4) TWWh1(2,4) TWC(2,4) TWWh2(2,4) TWC(2,4) TWWh3(2,4) TWC(2,4) TWWh4(2,4)],...
          [TWC(3,4) TWWh1(3,4) TWC(3,4) TWWh2(3,4) TWC(3,4) TWWh3(3,4) TWC(3,4) TWWh4(3,4)], 'Color', [0.4 0.4 0.4], 'LineWidth', 2.5);

% Plotting the last base frame
h6 = plotFrame(TWC, 0.3);
       
% Plotting starting and goal ee poses
h7 = plot3(xei,yei,zei, 'MarkerSize', 20, 'Marker', '.', 'Color', [0.1 0.1 0.1]);
h8 = plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', [0.1 0.1 0.1]);

% Plotting the reference frame
h9 = plotFrame([1 0 0 0;
                0 1 0 0;
                0 0 1 0;
                0 0 0 1], 1, 'W');
            

% Plotting scenario
h10 = contourf(X,Y,dilatedObstMap+obstMap,0:2);

% Plotting ee path
h11 = plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 1, 'Color', 'y');

% Plotting base path
h12 = plot3(x(10,1:end-1),x(11,1:end-1),zGC*ones(size(x,2)-1), 'LineWidth', 1.5, 'Color', [1,0.5,0]);

% Plotting reference path
h13 = plot3(x0(10,:),x0(11,:), zGC*ones(timeSteps,2), 'LineWidth', 1, 'Color', [0,0,0.6]);

% Plotting starting reference path
h14 = plot3(referencePath(1,:),referencePath(2,:), zGC*ones(timeSteps,2), 'LineWidth', 1, 'Color', [0,0,1]);

title('Mobile manipulator trajectories', 'interpreter', ...
'latex','fontsize',18)
daspect([1 1 1])
     
hold off;

%% SLQR algorithm
if stepped < 2
    state = 0;
    first = 1;
else
    state = 1;
    first = 0;
end

iter = 1;
while 1
    
    % Updating the plot
    if dynamicPlotting
        figure(1);
        
        % Plotting last arm config
        [TB0, TB1, TB2, TB3, TB4, TB5] = realDirect5(x(16:20,end-1));
        TWC = getTraslation([x(10,end-1),x(11,end-1),zGC])*getZRot(x(12,end-1));
        TW0 = TWC*TCB*TB0;
        TWWh1 = TWC*TCB*TB1;
        TWWh2 = TWC*TCB*TB2;
        TWWh3 = TWC*TCB*TB3;
        TWWh4 = TWC*TCB*TB4;
        TW5 = TWC*TCB*TB5;
        set(h4,'XData',[TW0(1,4) TWWh1(1,4) TWWh2(1,4) TWWh3(1,4) TWWh4(1,4) TW5(1,4)],...
               'YData',[TW0(2,4) TWWh1(2,4) TWWh2(2,4) TWWh3(2,4) TWWh4(2,4) TW5(2,4)],...
               'ZData',[TW0(3,4) TWWh1(3,4) TWWh2(3,4) TWWh3(3,4) TWWh4(3,4) TW5(3,4)],...
               'Color', [0.8 0.8 0.8], 'LineWidth', 2.5);

        % Plotting last rover position
        TWC = getTraslation([x(10,end-1),x(11,end-1),zGC])*getZRot(x(12,end-1));
        TCWh1 = getTraslation([dfy,dfx,-zGC]);
        TCWh2 = getTraslation([-dfy,dfx,-zGC]);
        TCWh3 = getTraslation([-dfy,-dfx,-zGC]);
        TCWh4 = getTraslation([dfy,-dfx,-zGC]);
        TWWh1 = TWC*TCWh1;
        TWWh2 = TWC*TCWh2;
        TWWh3 = TWC*TCWh3;
        TWWh4 = TWC*TCWh4;
        set(h5,'XData',[TWC(1,4) TWWh1(1,4) TWC(1,4) TWWh2(1,4) TWC(1,4) TWWh3(1,4) TWC(1,4) TWWh4(1,4)],...
               'YData',[TWC(2,4) TWWh1(2,4) TWC(2,4) TWWh2(2,4) TWC(2,4) TWWh3(2,4) TWC(2,4) TWWh4(2,4)],...
               'ZData',[TWC(3,4) TWWh1(3,4) TWC(3,4) TWWh2(3,4) TWC(3,4) TWWh3(3,4) TWC(3,4) TWWh4(3,4)],...
               'Color', [0.4 0.4 0.4], 'LineWidth', 2.5);

        % Plotting the last base frame
        h6 = plotFrame(TWC, 0.3, 0, h6);
        
                       
        % Plotting ee path
        set(h11,'XData',x(1,:),'YData',x(2,:),'ZData',x(3,:),...
            'LineWidth', 1, 'Color', 'y');

        % Plotting base path
        set(h12,'XData',x(10,1:end-1),'YData',x(11,1:end-1),'ZData',zGC*ones(size(x,2)-1,1),...
            'LineWidth', 1.5, 'Color', [1,0.5,0]);

        % Plotting reference path
        set(h13,'XData',x0(10,:),'YData',x0(11,:),'ZData',zGC*ones(timeSteps,1),...
            'LineWidth', 1, 'Color', [0,0,0.6]);

        hold off;       
    end   

    % Update the reference trajectory
    if updateReference
        x0 = MCM(x, x0, map, stateSpaceModel, trajectoryInfo, config);
    end
        
    % Quadratize cost function along the trajectory
    Q = zeros(numStates,numStates,timeSteps);

    for i = 1:timeSteps
        Q(10:12,10:12,i) = eye(3,3)*rtc;
        
%         if norm([x(10,i) x(11,i)] - [xef yef]) < reachabilityDistance
%             Q(10:12,10:12,i) = Q(10:12,10:12,i)/9999;
%         end
        if i > timeSteps*startBaseSpeedReduction/100
            linearCost = 10/timeSteps*i - startBaseSpeedReduction/10;
            Q(13,13,i) = linearCost*fsc;
            Q(14,14,i) = linearCost*fsc;
            Q(15,15,i) = linearCost*fsc;
        end
    end  
    
    Q(31,31,:) = tau1c;
    Q(32,32,:) = tau2c;
    Q(33,33,:) = tau3c;
    Q(34,34,:) = tau4c;
    Q(35,35,:) = tau5c;
    
    Q(40,40,:) = tauWheel;
    Q(41,41,:) = tauWheel;
       
    Q(1,1,end) = fc;
    Q(2,2,end) = fc;
    Q(3,3,end) = fc;
%     Q(7,7,end) = foc;
    Q(8,8,end) = foc;
%     Q(9,9,end) = foc;
    Q(16,16,end) = foc;
    Q(20,20,end) = foc;
    
    R = zeros(numInputs,numInputs,timeSteps);
    R(1,1,:) = ac1;
    R(2,2,:) = ac2;
    R(3,3,:) = ac3;
    R(4,4,:) = ac4;
    R(5,5,:) = ac5;
    R(6,6,:) = bc;
    R(7,7,:) = bc;  
    
    K = zeros(numStates,numInputs,timeSteps);
    
    quadratizedCost.Q = Q;
    quadratizedCost.R = R;
    quadratizedCost.K = K;

    % Linearize the system dynamics and constraints along the trajectory  
    Jac = zeros(6,5,timeSteps);
    for i = 2:timeSteps
        Jac(:,:,i-1) = jacobian5(x(16:20,i-1));
    end
    
     % State (x) matrix
    A = zeros(numStates,numStates,timeSteps);

    % W2EEx
    A(1,6,1) = cos(x(12,1));
    A(1,5,1) = -sin(x(12,1));
    A(1,10,1) = 1;

    % W2EEy
    A(2,6,1) = sin(x(12,1));
    A(2,5,1) = cos(x(12,1));
    A(2,11,1) = 1;

    % W2EEz
    A(3,3,1) = 1;

    % B2EE
    A(4:9,4:9,1) = eye(6,6);

    % W2Cx
    A(10,10,1) = 1;
    A(10,12,1) = dt*(-sin(x(12,1))*x(13,1)*kappa1-cos(x(12,1))*x(14,1)*kappa1);
    A(10,13,1) = dt*(cos(x(12,1))+sin(x(12,1))*x(12,1)*kappa1);
    A(10,14,1) = -dt*(sin(x(12,1))-cos(x(12,1))*x(12,1)*kappa1);

    % W2Cy
    A(11,11,1) = 1;
    A(11,12,1) = dt*(cos(x(12,1))*x(13,1)*kappa1-sin(x(12,1))*x(14,1)*kappa1);
    A(11,13,1) = dt*(sin(x(12,1))-cos(x(12,1))*x(12,1)*kappa1);
    A(11,14,1) = dt*(cos(x(12,1))+sin(x(12,1))*x(12,1)*kappa1);

    % W2C Heading
    A(12,12,1) = 1;
    A(12,15,1) = dt;
            
    % Arm joints Position
    A(16:20,16:20,1) = eye(5,5);
        
    % Arm joints acceleration
    A(26:30,21:25,1) = -1/dt*eye(5,5);
    
    % Arm joints torques
    A(31:35,26:30,1) = getB5(x(16:20,1));
    
%     [~,dG] = getG5(x(16:20,1));
%     A(31:35,16:20,1) = dG;
    
    % Wheels accelerations
    A(38:39,36:37,1) = -eye(2,2)/dt;
    
    % Wheels torques
    A(40:41,38:39,1) = eye(2,2)*(getWheelInertia(wheelMass,wheelRadius)+...
                                 vehicleMass/4*wheelRadius*wheelRadius);
                             
    for i = 2:timeSteps
        % W2EEx
        A(1,6,i) = cos(x(12,i-1));
        A(1,5,i) = -sin(x(12,i-1));
        A(1,10,i) = 1;

        % W2EEy
        A(2,6,i) = sin(x(12,i-1));
        A(2,5,i) = cos(x(12,i-1));
        A(2,11,i) = 1;

        % W2EEz
        A(3,3,i) = 1;

        % B2EE
        A(4:9,4:9,i) = eye(6,6);

        % W2Cx
        A(10,10,i) = 1;
        A(10,12,i) = dt*(-sin(x(12,i-1))*x(13,i-1)*kappa1-cos(x(12,i-1))*x(14,i-1)*kappa1);
        A(10,13,i) = dt*(cos(x(12,i-1))+sin(x(12,i-1))*x(12,i-1)*kappa1);
        A(10,14,i) = -dt*(sin(x(12,i-1))-cos(x(12,i-1))*x(12,i-1)*kappa1);

        % W2Cy
        A(11,11,i) = 1;
        A(11,12,i) = dt*(cos(x(12,i-1))*x(13,i-1)*kappa1-sin(x(12,i-1))*x(14,i-1)*kappa1);
        A(11,13,i) = dt*(sin(x(12,i-1))-cos(x(12,i-1))*x(12,i-1)*kappa1);
        A(11,14,i) = dt*(cos(x(12,i-1))+sin(x(12,i-1))*x(12,i-1)*kappa1);

        % W2C Heading
        A(12,12,i) = 1;
        A(12,15,i) = dt;
                  
        % Arm Joints Position
        A(16:20,16:20,i) = eye(5,5);
        
        % Arm joints acceleration
        A(26:30,21:25,i) = -1/dt*eye(5,5);
        
        % Arm joints torques
        A(31:35,26:30,i) = getB5(x(16:20,i-1));
        
%         [~,dG] = getG5(x(16:20,i-1));
%         A(31:35,16:20,i) = dG;
        
        % Wheels accelerations
        A(38:39,36:37,i) = -eye(2,2)/dt;

        % Wheels torques
        A(40:41,38:39,i) = eye(2,2)*(getWheelInertia(wheelMass,wheelRadius)+...
                                     vehicleMass/4*wheelRadius*wheelRadius);

    end
    
    % Actuation (u) matrix
    B = zeros(numStates,numInputs,timeSteps);
    
    % WTEEz
    B(3,1:5,1) = -dt*Jac(1,:,1);
    
    % BTEE
    B(4:9,1:5,1) = dt*Jac(:,:,1);

    % W2C Speed x
    B(13,6,1) = wheelRadius/2;
    B(13,7,1) = wheelRadius/2;
        
    % W2C Speed heading
    B(15,6,1) = wheelRadius/(2*dfx);
    B(15,7,1) = -wheelRadius/(2*dfx);
    
    % Arm joints Position
    B(16:20,1:5,1) = dt*eye(5,5);
    
    % Arm joints speed
    B(21:25,1:5,1) = eye(5,5);
    
    % Arm joints acceleration
    B(26:30,1:5,1) = 1/dt*eye(5,5);

    % Arm joints torques
    B(31:35,1:5,1) = getC5(x(16:20,1), u(1:5,1));
    
    % Wheels speeds
    B(36,6,1) = 1;
    B(37,7,1) = 1;
    
    % Wheels acceleration
    B(38,6,1) = 1/dt;
    B(39,7,1) = 1/dt;
    
    for i = 2:timeSteps
        % WTEEz
        B(3,1:5,i) = -dt*Jac(1,:,i-1);
        
        % BTEE
        B(4:9,1:5,i) = dt*Jac(:,:,i-1);

        % W2C Speed x
        B(13,6,i) = wheelRadius/2;
        B(13,7,i) = wheelRadius/2;

        % W2C Speed heading
        B(15,6,i) = wheelRadius/(2*dfx);
        B(15,7,i) = -wheelRadius/(2*dfx);

        % Arm joints Position
        B(16:20,1:5,i) = dt*eye(5,5);

        % Arm joints speed
        B(21:25,1:5,i) = eye(5,5);

        % Arm joints acceleration
        B(26:30,1:5,i) = 1/dt*eye(5,5);

        % Arm joints torques
        B(31:35,1:5,i) = getC5(x(16:20,i-1),...
                               u(1:5,i-1));

        % Wheels speeds
        B(36,6,i) = 1;
        B(37,7,i) = 1;

        % Wheels acceleration
        B(38,6,i) = 1/dt;
        B(39,7,i) = 1/dt;

    end
    
    stateSpaceModel.A = A;
    stateSpaceModel.B = B;
    
    % Checking the state
    switch state
        case 0
            [x, u, converged] = SLQ(x, x0, u, u0, dt,...
                        stateSpaceModel, quadratizedCost, config, map);

            % Check whether the algorithm has finished
            if converged > 0
                disp(['SLQ found the unconstrained optimal control input within ',num2str(iter-1),' iterations'])
                toc;
                if stepped == 0
                    warning('The constraints will not be checked, modify the configuration property "stepped" to change this behaviour')
                    break;
                else
                    constraintsSatisfied = checkConstraints(x, u, stateSpaceModel);
                    if constraintsSatisfied 
                        disp('The imposed constraints are satisfied, the final control input is found')
                        break;
                    else
                        disp('Since the constraints are not satisfied yet, a further constrained SLQ will be performed')
                        state = 1;
                    end
                end
            end
        case 1
            if first
                % Solving the inequality constrained SLQR problem
                xs = x - xini;
                us = u - uini;
                [x, u, stateSpaceModel.I, stateSpaceModel.J, converged] = ...
                constrainedSLQ(xini,x0,xs,uini,u0,us,dt,stateSpaceModel,quadratizedCost,config, map);
                first = 0;
            else
                % Solving the inequality constrained SLQR problem
                [x, u, stateSpaceModel.I, stateSpaceModel.J, converged] = ...
                constrainedSLQ(x,x0,u,u0,dt,stateSpaceModel,quadratizedCost,config, map);
            end
            
            % Check whether the algorithm has finished
            if converged > 0
                disp('The imposed constraints are satisfied, the final control input is found')
                disp(['Constrained SLQ refined control input within ',num2str(iter-1),' iterations'])
                break;
            end
        otherwise
        % This case is not reachable
    end   

    
    disp(['Iteration number ',num2str(iter),...
          ', distance to goal = ',num2str(norm(x(1:3,end)-x0(1:3,end)))]);
      
    disp(['Is the path safe? ', num2str(isSafePath(x(10,:),x(11,:),mapResolution,dilatedObstMap))])    
      
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
% Plotting stuff
for i = 2:timeSteps
    if(x(16,i) < armJointsPositionLimits(1,1) || x(16,i) > armJointsPositionLimits(1,2))
        warning(['Arm joint 1 is violating its position limits at waypoint ',num2str(i)]);
    end
    if(x(17,i) < armJointsPositionLimits(2,1) || x(17,i) > armJointsPositionLimits(2,2))
        warning(['Arm joint 2 is violating its position limits at waypoint ',num2str(i)]);
    end
    if(x(18,i) < armJointsPositionLimits(3,1) || x(18,i) > armJointsPositionLimits(3,2))
        warning(['Arm joint 3 is violating its position limits at waypoint ',num2str(i)]);
    end
    if(x(19,i) < armJointsPositionLimits(3,1) || x(18,i) > armJointsPositionLimits(3,2))
        warning(['Arm joint 3 is violating its position limits at waypoint ',num2str(i)]);
    end      
    if(x(20,i) < armJointsPositionLimits(3,1) || x(18,i) > armJointsPositionLimits(3,2))
        warning(['Arm joint 3 is violating its position limits at waypoint ',num2str(i)]);
    end      
end

iu = cumsum(abs(x(31,:))*dt);
disp(['Total torque applied arm joint 1: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(32,:))*dt);
disp(['Total torque applied arm joint 2: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(33,:))*dt);
disp(['Total torque applied arm joint 3: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(34,:))*dt);
disp(['Total torque applied arm joint 4: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(35,:))*dt);
disp(['Total torque applied arm joint 5: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(40,:))*dt);
disp(['Total torque applied left wheels: ',num2str(iu(end)),' Nm'])
iu = cumsum(abs(x(41,:))*dt);
disp(['Total torque applied right wheels: ',num2str(iu(end)),' Nm'])

figure(1);        
% Plotting last arm config
[TB0, TB1, TB2, TB3, TB4, TB5] = realDirect5(x(16:20,end-1));
TWC = getTraslation([x(10,end-1),x(11,end-1),zGC])*getZRot(x(12,end-1));
TW0 = TWC*TCB*TB0;
TWWh1 = TWC*TCB*TB1;
TWWh2 = TWC*TCB*TB2;
TWWh3 = TWC*TCB*TB3;
TWWh4 = TWC*TCB*TB4;
TW5 = TWC*TCB*TB5;
set(h4,'XData',[TW0(1,4) TWWh1(1,4) TWWh2(1,4) TWWh3(1,4) TWWh4(1,4) TW5(1,4)],...
       'YData',[TW0(2,4) TWWh1(2,4) TWWh2(2,4) TWWh3(2,4) TWWh4(2,4) TW5(2,4)],...
       'ZData',[TW0(3,4) TWWh1(3,4) TWWh2(3,4) TWWh3(3,4) TWWh4(3,4) TW5(3,4)],...
       'Color', [0.8 0.8 0.8], 'LineWidth', 2.5);

% Plotting last rover position
TWC = getTraslation([x(10,end-1),x(11,end-1),zGC])*getZRot(x(12,end-1));
TCWh1 = getTraslation([dfy,dfx,-zGC]);
TCWh2 = getTraslation([-dfy,dfx,-zGC]);
TCWh3 = getTraslation([-dfy,-dfx,-zGC]);
TCWh4 = getTraslation([dfy,-dfx,-zGC]);
TWWh1 = TWC*TCWh1;
TWWh2 = TWC*TCWh2;
TWWh3 = TWC*TCWh3;
TWWh4 = TWC*TCWh4;
set(h5,'XData',[TWC(1,4) TWWh1(1,4) TWC(1,4) TWWh2(1,4) TWC(1,4) TWWh3(1,4) TWC(1,4) TWWh4(1,4)],...
       'YData',[TWC(2,4) TWWh1(2,4) TWC(2,4) TWWh2(2,4) TWC(2,4) TWWh3(2,4) TWC(2,4) TWWh4(2,4)],...
       'ZData',[TWC(3,4) TWWh1(3,4) TWC(3,4) TWWh2(3,4) TWC(3,4) TWWh3(3,4) TWC(3,4) TWWh4(3,4)],...
       'Color', [0.4 0.4 0.4], 'LineWidth', 2.5);

% Plotting the last base frame
h6 = plotFrame(TWC, 0.3, 0, h6);


% Plotting ee path
set(h11,'XData',x(1,:),'YData',x(2,:),'ZData',x(3,:),...
    'LineWidth', 1, 'Color', 'y');

% Plotting base path
set(h12,'XData',x(10,1:end-1),'YData',x(11,1:end-1),'ZData',zGC*ones(size(x,2)-1,1),...
    'LineWidth', 1.5, 'Color', [1,0.5,0]);

% Plotting reference path
set(h13,'XData',x0(10,:),'YData',x0(11,:),'ZData',zGC*ones(timeSteps,1),...
    'LineWidth', 1, 'Color', [0,0,0.6]);

hold off; 
        
% figure(2)
% plot(t,x(16:20,:))
% title('Evolution of the arm joints', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\theta_1$','$\theta_2$','$\theta_3$','$\theta_4$','$\theta_5$', 'interpreter', ...
% 'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
% grid 
% 
% figure(3)
% plot(t,u(1:5,:))
% title('Evolution of the arm joints speed', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\dot\theta_1$','$\dot\theta_2$','$\dot\theta_3$','$\dot\theta_4$','$\dot\theta_5$', 'interpreter', ...
% 'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\dot\theta (rad/s)$', 'interpreter', 'latex','fontsize',18)
% grid 
% 
figure(4)
plot(t,u(6:7,:))
title('Actuating wheels speed','interpreter','latex')
xlabel('t(s)','interpreter','latex','fontsize',18)
ylabel('$\dot\theta(rad/s$)','interpreter','latex','fontsize',18)
legend('$\omega_r$','$\dot\omega_l$','interpreter', ...
'latex','fontsize',18)
% 
% figure(5)
% plot(t,x(26:30,:))
% title('Evolution of the arm joints accelerations', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\ddot\theta_1$','$\ddot\theta_2$','$\ddot\theta_3$', 'interpreter', ...
% 'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\ddot\theta (rad/s^2)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
% figure(6)
% plot(t,x(31:35,:))
% title('Evolution of the applied arm torques', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\tau_1$','$\tau_2$','$\tau_3$', 'interpreter', ...
% 'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\tau (Nm)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
% figure(7)
% plot(t,x(36:37,:))
% title('Evolution of the applied wheel speeds', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\omega 1$','$\omega 2$','interpreter','latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\omega (rad/s)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
% figure(8)
% plot(t,x(38:39,:))
% title('Evolution of the applied wheel accelerations', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\dot\omega 1$','$\dot\omega 2$','$\dot\omega 3$',...
% '$\dot\omega 4$', 'interpreter','latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\dot\omega (rad/s^2)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
% figure(9)
% plot(t,x(40:41,:))
% hold on
% yline(wheelTorqueLimit,'--');
% yline(-wheelTorqueLimit,'--');
% title('Evolution of the applied wheel torques', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$\tau_{\omega 1}$','$\tau_{\omega 2}$', 'interpreter','latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\tau (Nm)$', 'interpreter', 'latex','fontsize',18)
% grid
% hold off
% 
% figure(10)
% plot(t,x(12,:))
% title('Evolution of the vehicle heading', 'interpreter', ...
% 'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\psi (rad)$', 'interpreter', 'latex','fontsize',18)
% grid
% 
% figure(11)
% plot(t,x(7:9,:))
% title('Evolution of the arm orientation', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$Roll$','$Pitch$', '$Yaw$', 'interpreter','latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% ylabel('$\psi (rad)$', 'interpreter', 'latex','fontsize',18)
% grid

figure(12)
plot(t,x(13:15,:))
title('Evolution of the rover base speed', 'interpreter', ...
'latex','fontsize',18)
legend('$\dot x_B$','$\dot y_B$', '$\dot \theta_B$', 'interpreter','latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
ylabel('$Speed (m/s)$', 'interpreter', 'latex','fontsize',18)
grid

%% Simulation

% Model parameters
l = 0.125;              % length of legs
Bm = 11*10^-5;          % rotational damper
Jw = 12*10^-4;          % moment of inertia of the wheels
theta = 0;              % initial angle of the joints
rho = 2700;             % links density

% PI parameters of the wheel speed controller
kp = 150;               
ki = 50;

% Terrain parameters
mu = 0.02;               % friction
s = 0.0;                % slip ratio 
slope = 0;              % slope

% sim('ExoTeR',t(end));



