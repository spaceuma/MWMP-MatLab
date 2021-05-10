%% Initialization

addpath('../../../ARES-DyMu_matlab/Global Path Planning/functions')
addpath('../../maps')
addpath('../../models')
addpath('../../costs')
addpath('../../utils')
 
clear

tic
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

global dfx;
dfx = 0.7;
global dfy;
dfy = 0.7;
global r;
r = 0.2;
global zBC;
zBC = 0.645;

global reachabilityDistance;
reachabilityDistance = (a1+a2+d4+d6-zBC-d0);

riskDistance = 1;
safetyDistance = 1.5;
mapResolution = 0.05;
vehicleSpeed = 0.1;

armJointsLimits = [-360 +360;
                   -120  80;
                   -140 +140;
                   -360 +360;
                   -135 +135;
                   -360 +360]*pi/180;

%% Constraints 
xB0 = 2.00;
yB0 = 2.20;
zB0 = zBC;
yawB0 = pi;

qi = [0, -pi/2, pi/2, 0, pi/2, 0];
rollei = 0;
pitchei = pi;
yawei = 0;

xef = 2.40;
yef = 8.65;
zef = 0.2;
rollef = 0;
pitchef = pi;
yawef = 0;

TWB = getTraslation([xB0,yB0,zB0])*getZRot(yawB0);
[~, ~, ~, ~, ~, ~, TB6] = direct(qi);

TW6 = TWB*TB6;

xei = TW6(1,4);
yei = TW6(2,4);
zei = TW6(3,4);

%% Costs
% State costs
fc = 1000000; % Final state cost, 1000000
foc = 1000000; % Final orientation cost, 1000000
fsc = 1000000; % Final zero speed cost, 1000000
rtc = 1.0; % Reference path max cost 1
rtor = 0.25; % Percentage of rtc when wayp orientation = pi/2

% Input costs
bc = 0.1; % Base actuation cost, 0.1
sc = 0.1; % Steering cost, 0.1
ac = 0.1; % Arm actuation cost, 0.1

% Extra costs
sm = 50; % Influence of diff turns into final speed, tune till convergence 50
sm2 = 9999999999; % Influence of steer turns into final speed, tune till convergence 999999999
tc = 0.000; % Total cost map cost, 0.005
% tco = 0.01; % Total cost map orientation cost, 1.0

tf = 60; % Time vector
dt = 0.6;
t = 0:dt:tf;

distThreshold = 0.031; % When should we stop the algorithm...? (metres)

lineSearchStep = 0.10; % Step actuation percentage

% iterFCApproaching = 0;

maxIter = 500; % Maximum number of iterations

tau = 0.5; % GDM step size (1 = mapResolution)


%% Algorithm
% FMM to compute totalCostMap
load('obstMap3','obstMap')
dilatedObstMap = dilateObstMap(obstMap, riskDistance, mapResolution);
safeObstMap = dilateObstMap(obstMap, safetyDistance, mapResolution);

costMap = ones(size(obstMap));

distMap = mapResolution*bwdist(obstMap);
auxMap = 1./distMap;
gradient = 5;
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

% Resize path
x1 = 1:size(referencePath,1);
x2 = linspace(1,size(referencePath,1),size(t,2));
referencePath = interp1(x1,referencePath,x2);

yaw = getYaw(referencePath);
referencePath = [referencePath yaw].';

% State vectors
x = zeros(25,size(t,2));
% WTEE
x(1,1) = xei;
x(2,1) = yei;
x(3,1) = zei;
% BTEE
x(4,1) = TB6(1,4);
x(5,1) = TB6(2,4);
x(6,1) = TB6(3,4);
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
% ArmJoints
x(16,1) = qi(1);
x(17,1) = qi(2);
x(18,1) = qi(3);
x(19,1) = qi(4);
x(20,1) = qi(5);
x(21,1) = qi(6);
% Steering joints
x(22,1) = 0;
x(23,1) = 0;
x(24,1) = 0;
x(25,1) = 0;

% Initial control law
u = zeros(10,size(t,2));

% Target state and control trajectories
x0 = zeros(25,size(t,2));

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
% ArmJoints
x0(16,end) = 0;
x0(17,end) = 0;
x0(18,end) = 0;
x0(19,end) = 0;
x0(20,end) = 0;
x0(21,end) = 0;
% Steering joints
x0(22,end) = 0;
x0(23,end) = 0;
x0(24,end) = 0;
x0(25,end) = 0;

u0 = zeros(10,size(t,2));

Jac = zeros(6,6,size(t,2));

% GDM initialization
reachabilityIndex = 0;

% Plotting stuff
map = [0 0.6   0
       0.6 0.3 0
       0.6 0   0];
colormap(map);
xVect = linspace(0,9.95,200);
[X,Y] = meshgrid(xVect,xVect);

% SLQR algorithm
iter = 1;
error = 0;
while 1   
    % Forward integrate system equations
    for i = 2:size(t,2)
        Jac(:,:,i-1) = jacobian(x(16:21,i-1));
        % W2EE
        x(1,i) = cos(x(12,i-1))*x(4,i-1) - sin(x(12,i-1))*x(5,i-1) + x(10,i-1);
        x(2,i) = sin(x(12,i-1))*x(4,i-1) + cos(x(12,i-1))*x(5,i-1) + x(11,i-1);
        x(3,i) = x(6,i-1) + zBC;
        % B2EE
        x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*u(1:6,i-1)*dt; 
        % W2B
        x(10,i) = x(10,i-1) + cos(x(12,i-1))*x(13,i-1)*dt - sin(x(12,i-1))*x(14,i-1)*dt;
        x(11,i) = x(11,i-1) + sin(x(12,i-1))*x(13,i-1)*dt + cos(x(12,i-1))*x(14,i-1)*dt;
        x(12,i) = x(12,i-1) + x(15,i-1)*dt;
        % Bspeed
        x(13,i) = r/2*(cos(x(22,i-1))*u(7,i-1) + cos(x(24,i-1))*u(8,i-1));
        x(14,i) = - r/2*(sin(x(22,i-1))*u(7,i-1) + sin(x(24,i-1))*u(8,i-1));
        x(15,i) = r/(2*dfx)*(cos(x(22,i-1))*u(7,i-1) - cos(x(24,i-1))*u(8,i-1));
        % Arm Joints Position
        x(16:21,i) = x(16:21,i-1) + u(1:6,i-1)*dt;
        % Steering Joints Position
        x(22:23,i) =  x(22:23,i-1) + u(9,i-1)*dt;
        x(24:25,i) =  x(24:25,i-1) + u(10,i-1)*dt;
    end
    Jac(:,:,end) = jacobian(x(16:21,end));

    
    % Multitrajectory costing method
    for i = 2:size(t,2)-2
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
        
        % Resize path
        x1 = 1:size(pathi,1);
        x2 = linspace(1,size(pathi,1),size(t,2)-i+1);
        pathi = interp1(x1,pathi,x2);
        
        yaw = getYaw(pathi);
        pathi = [pathi yaw].';
        
         
        if norm(x(10:11,i) - x(10:11,i-1)) > tau*mapResolution && ...
            norm(x(10:11,i) - x0(10:11,i)) < 2
            newCost = getTrajectoryCost(pathi, x(10:12,i:end));
            oldCost = getTrajectoryCost(x0(10:12,i:end), x(10:12,i:end));

            if newCost*1.30 < oldCost && oldCost > 0.05 &&...
               isSafePath(pathi(1,:),pathi(2,:),mapResolution,dilatedObstMap) && ...
               DiscreteFrechetDist(pathi, x0(10:12,i:end)) > 0.5
                x0(10:12,i:end) = pathi;
                disp(['Changing reference path from waypoint ',num2str(i), '...'])
            end
        end

    end    
    
    % Update reference trajectories
    xh0 = x0 - x;
    uh0 = u0 - u;      
    
    % Quadratize cost function along the trajectory
    Q = zeros(size(x,1),size(x,1),size(t,2));
    for i = 1:size(t,2)
        [Tcmx, Tcmy] = getGradientTotalCost(x(10,i), x(11,i), mapResolution, gTCMx, gTCMy);
        descYaw = atan2(-Tcmy, -Tcmx);
        
        headingDeviation = abs(x(12,i) - descYaw);
        while headingDeviation > 2*pi
            headingDeviation = abs(headingDeviation - 2*pi);
        end
        
        Q(10:12,10:12,i) = (rtc - rtc*rtor/pi*(3/rtor-4)*headingDeviation + 2*rtc*rtor*(1/rtor-2)/pi^2 * headingDeviation^2)*eye(3,3);
        
        if norm([x(10,i) x(11,i)] - [xef yef]) < reachabilityDistance
            Q(10:12,10:12,i) = Q(10:12,10:12,i)/9999;
        end
    end
    
    Qend = zeros(size(x,1),size(x,1));
    
    Qend(1,1) = fc;
    Qend(2,2) = fc;
    Qend(3,3) = fc;
    Qend(7,7) = foc;
    Qend(8,8) = foc;
    Qend(9,9) = 0;
    Qend(13,13) = fsc;
    Qend(14,14) = fsc;
    Qend(15,15) = fsc;
    
    R = eye(size(u,1));
    R(1,1) = ac;
    R(2,2) = ac;
    R(3,3) = ac;
    R(4,4) = ac;
    R(5,5) = ac;
    R(6,6) = ac;
    R(7,7) = bc;
    R(8,8) = bc;  
    R(9,9) = sc;
    R(10,10) = sc;    
       
    % Linearize the system dynamics and constraints along the trajectory  
    
    % State (x) matrix
    A = zeros(size(x,1),size(x,1),size(t,2));

    % W2EEx
    A(1,4,1) = cos(x(12,1));
    A(1,5,1) = -sin(x(12,1));
    A(1,10,1) = 1;

    % W2EEy
    A(2,4,1) = sin(x(12,1));
    A(2,5,1) = cos(x(12,1));
    A(2,11,1) = 1;

    % W2EEz
    A(3,6,1) = 1;

    % B2EE
    A(4:9,4:9,1) = eye(6,6);

    % W2Bx
    A(10,10,1) = 1;
    A(10,12,1) = dt*(-sin(x(12,1))*x(13,1)/sm-cos(x(12,1))*x(14,1)/sm);
    A(10,13,1) = dt*(cos(x(12,1))+sin(x(12,1))*x(12,1)/sm);
    A(10,14,1) = -dt*(sin(x(12,1))-cos(x(12,1))*x(12,1)/sm);

    % W2By
    A(11,11,1) = 1;
    A(11,12,1) = dt*(cos(x(12,1))*x(13,1)/sm-sin(x(12,1))*x(14,1)/sm);
    A(11,13,1) = dt*(sin(x(12,1))-cos(x(12,1))*x(12,1)/sm);
    A(11,14,1) = dt*(cos(x(12,1))+sin(x(12,1))*x(12,1)/sm);

    % W2B Heading
    A(12,12,1) = 1;
    A(12,15,1) = dt;

    % W2B Speed x
    A(13,22,1) = r/2*(-sin(x(22,1))*u(7,1))/sm2;
    A(13,24,1) = r/2*(-sin(x(24,1))*u(8,1))/sm2;
    
    % W2B Speed y
    A(14,22,1) = -r/2*cos(x(22,1))*u(7,1)/sm2;
    A(14,24,1) = -r/2*cos(x(24,1))*u(8,1)/sm2;
    
    % W2B Speed Heading
    A(15,22,1) = r/(2*dfx) *(-sin(x(22,1))*u(7,1))/sm2;
    A(15,24,1) = -r/(2*dfx)*(-sin(x(24,1))*u(8,1))/sm2;
    
    % Arm Joints Position
    A(16:21,16:21,1) = eye(6,6);

    % Steering Joints Position
    A(22:25,22:25,1) = eye(4,4);
    
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
        A(3,6,i) = 1;

        % B2EE
        A(4:9,4:9,i) = eye(6,6);

        % W2Bx
        A(10,10,i) = 1;
        A(10,12,i) = dt*(-sin(x(12,i-1))*x(13,i-1)/sm-cos(x(12,i-1))*x(14,i-1)/sm);
        A(10,13,i) = dt*(cos(x(12,i-1))+sin(x(12,i-1))*x(12,i-1)/sm);
        A(10,14,i) = -dt*(sin(x(12,i-1))-cos(x(12,i-1))*x(12,i-1)/sm);

        % W2By
        A(11,11,i) = 1;
        A(11,12,i) = dt*(cos(x(12,i-1))*x(13,i-1)/sm-sin(x(12,i-1))*x(14,i-1)/sm);
        A(11,13,i) = dt*(sin(x(12,i-1))-cos(x(12,i-1))*x(12,i-1)/sm);
        A(11,14,i) = dt*(cos(x(12,i-1))+sin(x(12,i-1))*x(12,i-1)/sm);

        % W2B Heading
        A(12,12,i) = 1;
        A(12,15,i) = dt;

        % W2B Speed x
        A(13,22,i) = r/2*(-sin(x(22,i-1))*u(7,i-1))/sm2;
        A(13,24,i) = r/2*(-sin(x(24,i-1))*u(8,i-1))/sm2;

        % W2B Speed y
        A(14,22,i) = -r/2*cos(x(22,i-1))*u(7,i-1)/sm2;
        A(14,24,i) = -r/2*cos(x(24,i-1))*u(8,i-1)/sm2;

        % W2B Speed Heading
        A(15,22,i) = r/(2*dfx) *(-sin(x(22,i-1))*u(7,i-1))/sm2;
        A(15,24,i) = -r/(2*dfx)*(-sin(x(24,i-1))*u(8,i-1))/sm2;

        % Arm Joints Position
        A(16:21,16:21,i) = eye(6,6);

        % Steering Joints Position
        A(22:25,22:25,i) = eye(4,4);
    end
    
    % Actuation (u) matrix
    B = zeros(size(x,1),size(u,1),size(t,2));
    
    % B2EE
    B(4:9,1:6,1) = dt*Jac(:,:,1);

    % W2B Speed x
    B(13,7,1) = r/2*(cos(x(22,1)) + x(22,1)*sin(x(22,1))/sm2);
    B(13,8,1) = r/2*(cos(x(24,1)) + x(24,1)*sin(x(24,1))/sm2);
    
    % W2B Speed y
    B(14,7,1) = -r/2*(sin(x(22,1)) - x(22,1)*cos(x(22,1))/sm2);
    B(14,8,1) = -r/2*(sin(x(24,1)) - x(24,1)*cos(x(24,1))/sm2);
    
    % W2B Speed heading
    B(15,7,1) = r/(2*dfx)*(cos(x(22,1)) + x(22,1)*sin(x(22,1))/sm2);
    B(15,8,1) = -r/(2*dfx)*(cos(x(24,1)) + x(24,1)*sin(x(24,1))/sm2);
    
    % Arm Joints Position
    B(16:21,1:6,1) = dt*eye(6,6);
    
    % Steering Joints Position
    B(22:23,9,1) = dt;
    B(24:25,10,1) = dt;
    
    for i = 2:size(t,2)
        % B2EE
        B(4:9,1:6,i) = dt*Jac(:,:,i-1);

        % W2B Speed x
        B(13,7,i) = r/2*(cos(x(22,i-1)) + x(22,i-1)*sin(x(22,i-1))/sm2);
        B(13,8,i) = r/2*(cos(x(24,i-1)) + x(24,i-1)*sin(x(24,i-1))/sm2);

        % W2B Speed y
        B(14,7,i) = -r/2*(sin(x(22,i-1)) - x(22,i-1)*cos(x(22,i-1))/sm2);
        B(14,8,i) = -r/2*(sin(x(24,i-1)) - x(24,i-1)*cos(x(24,i-1))/sm2);

        % W2B Speed heading
        B(15,7,i) = r/(2*dfx)*(cos(x(22,i-1)) + x(22,1)*sin(x(22,i-1))/sm2);
        B(15,8,i) = -r/(2*dfx)*(cos(x(24,i-1)) + x(24,1)*sin(x(24,i-1))/sm2);

        % Arm Joints Position
        B(16:21,1:6,i) = dt*eye(6,6);

        % Steering Joints Position
        B(22:23,9,i) = dt;
        B(24:25,10,i) = dt;
    end    
    
    % Total cost map cost
    Tcmx = zeros(size(Q,1),size(t,2)); 
    if tc > 0
        for i = 1:size(t,2)
            [Tcmx(13,i), Tcmx(14,i)] = getGradientTotalCost(x(10,i), x(11,i), mapResolution, gTCMx, gTCMy);
            Tcmx(13,i) = tc*Tcmx(13,i);
            Tcmx(14,i) = tc*Tcmx(14,i);
        end
    end       
        
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
    
    P(:,:,end) = Qend;
    s(:,:,end) = -Qend*xh0(:,end) + Tcmx(:,end);
    
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
        s(:,:,i) = A(:,:,i).'*(eye(size(Q,1))...
            - P(:,:,i+1)*M(:,:,i)*B(:,:,i)*inv(R)*B(:,:,i).')*s(:,:,i+1)...
            + A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*B(:,:,i)*uh0(:,i)...
            - Q(:,:,i)*xh0(:,i) + Tcmx(:,i);
    end
    
    % Solve forward
    for i = 1:size(t,2)-1
        v(:,i) = M(:,:,i)*B(:,:,i)*(uh0(:,i)-inv(R)*B(:,:,i).'*s(:,:,i+1));
        xh(:,i+1) = M(:,:,i)*A(:,:,i)*xh(:,i)+v(:,i);
        lambdah(:,:,i+1) = P(:,:,i+1)*xh(:,i+1)+s(:,:,i+1);
        uh(:,i) = uh0(:,i)-inv(R)*B(:,:,i).'*lambdah(:,:,i+1);
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
            for i = 2:size(t,2)
                Jac(:,:,i-1) = jacobian(x(16:21,i-1));
                % W2EE
                x(1,i) = cos(x(12,i-1))*x(4,i-1) - sin(x(12,i-1))*x(5,i-1) + x(10,i-1);
                x(2,i) = sin(x(12,i-1))*x(4,i-1) + cos(x(12,i-1))*x(5,i-1) + x(11,i-1);
                x(3,i) = x(6,i-1) + zBC;
                % B2EE
                x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*u(1:6,i-1)*dt; 
                % W2B
                x(10,i) = x(10,i-1) + cos(x(12,i-1))*x(13,i-1)*dt - sin(x(12,i-1))*x(14,i-1)*dt;
                x(11,i) = x(11,i-1) + sin(x(12,i-1))*x(13,i-1)*dt + cos(x(12,i-1))*x(14,i-1)*dt;
                x(12,i) = x(12,i-1) + x(15,i-1)*dt;
                % Bspeed
                x(13,i) = r/2*(cos(x(22,i-1))*u(7,i-1) + cos(x(24,i-1))*u(8,i-1));
                x(14,i) = - r/2*(sin(x(22,i-1))*u(7,i-1) + sin(x(24,i-1))*u(8,i-1));
                x(15,i) = r/(2*dfx)*(cos(x(22,i-1))*u(7,i-1) - cos(x(24,i-1))*u(8,i-1));
                % Arm Joints Position
                x(16:21,i) = x(16:21,i-1) + u(1:6,i-1)*dt;
                % Steering Joints Position
                x(22:23,i) =  x(22:23,i-1) + u(9,i-1)*dt;
                x(24:25,i) =  x(24:25,i-1) + u(10,i-1)*dt;
            end
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
%         u = u + uh;

    end
    
    
    if iter > maxIter
        cprintf('err','SLQR failed to converge to a solution\n')
        error = 1;
        break;
    end
    
    figure(1)
    % Plotting first arm config
    [TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(x(16:21,1));
    TWB = getTraslation([x(10,1),x(11,1),zBC])*getZRot(x(12,1));
    TW0 = TWB*TB0;
    TW1 = TWB*TB1;
    TW2 = TWB*TB2;
    TW3 = TWB*TB3;
    TW4 = TWB*TB4;
    TW5 = TWB*TB5;
    TW6 = TWB*TB6;
    plot3([TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4) TW4(1,4) TW5(1,4) TW6(1,4)],...
          [TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4) TW4(2,4) TW5(2,4) TW6(2,4)],...
          [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4) TW4(3,4) TW5(3,4) TW6(3,4)], 'Color', [0.75, 0.75, 0.75], 'LineWidth', 2.5);
    hold on;
    
    % Plotting last arm config
    [TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(x(16:21,end-1));
    TWB = getTraslation([x(10,end-1),x(11,end-1),zBC])*getZRot(x(12,end-1));
    TW0 = TWB*TB0;
    TW1 = TWB*TB1;
    TW2 = TWB*TB2;
    TW3 = TWB*TB3;
    TW4 = TWB*TB4;
    TW5 = TWB*TB5;
    TW6 = TWB*TB6;
    plot3([TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4) TW4(1,4) TW5(1,4) TW6(1,4)],...
          [TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4) TW4(2,4) TW5(2,4) TW6(2,4)],...
          [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4) TW4(3,4) TW5(3,4) TW6(3,4)], 'Color', [0.75, 0.75, 0.75], 'LineWidth', 2.5);
      
    % Plotting first rover position
    TWB = getTraslation([x(10,1),x(11,1),zBC])*getZRot(x(12,1));
    TB1 = getTraslation([dfx,dfy,-zBC]);
    TB2 = getTraslation([-dfx,dfy,-zBC]);
    TB3 = getTraslation([-dfx,-dfy,-zBC]);
    TB4 = getTraslation([dfx,-dfy,-zBC]);
    TW1 = TWB*TB1;
    TW2 = TWB*TB2;
    TW3 = TWB*TB3;
    TW4 = TWB*TB4;
    plot3([TWB(1,4) TW1(1,4) TWB(1,4) TW2(1,4) TWB(1,4) TW3(1,4) TWB(1,4) TW4(1,4)],...
          [TWB(2,4) TW1(2,4) TWB(2,4) TW2(2,4) TWB(2,4) TW3(2,4) TWB(2,4) TW4(2,4)],...
          [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', [0.75, 0.75, 0.75], 'LineWidth', 2.5);
   
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
              [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', [0.75, 0.75, 0.75], 'LineWidth', 2.5);

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
    plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', 'c')

    hold off;

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
if error == 0
    for i = 2:size(t,2)
        Jac(:,:,i-1) = jacobian(x(16:21,i-1));
        % W2EE
        x(1,i) = cos(x(12,i-1))*x(4,i-1) - sin(x(12,i-1))*x(5,i-1) + x(10,i-1);
        x(2,i) = sin(x(12,i-1))*x(4,i-1) + cos(x(12,i-1))*x(5,i-1) + x(11,i-1);
        x(3,i) = x(6,i-1) + zBC;
        % B2EE
        x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*u(1:6,i-1)*dt; 
        % W2B
        x(10,i) = x(10,i-1) + cos(x(12,i-1))*x(13,i-1)*dt - sin(x(12,i-1))*x(14,i-1)*dt;
        x(11,i) = x(11,i-1) + sin(x(12,i-1))*x(13,i-1)*dt + cos(x(12,i-1))*x(14,i-1)*dt;
        x(12,i) = x(12,i-1) + x(15,i-1)*dt;
        % Bspeed
        x(13,i) = r/2*(cos(x(22,i-1))*u(7,i-1) + cos(x(24,i-1))*u(8,i-1));
        x(14,i) = - r/2*(sin(x(22,i-1))*u(7,i-1) + sin(x(24,i-1))*u(8,i-1));
        x(15,i) = r/(2*dfx)*(cos(x(22,i-1))*u(7,i-1) - cos(x(24,i-1))*u(8,i-1));
        % Arm Joints Position
        x(16:21,i) = x(16:21,i-1) + u(1:6,i-1)*dt;
        % Steering Joints Position
        x(22:23,i) =  x(22:23,i-1) + u(9,i-1)*dt;
        x(24:25,i) =  x(24:25,i-1) + u(10,i-1)*dt;

        if(x(16,i) < armJointsLimits(1,1) || x(16,i) > armJointsLimits(1,2))
            disp(['WARNING: Arm joint 1 is violating its position limits at waypoint ',num2str(i)]);
        end
        if(x(17,i) < armJointsLimits(2,1) || x(17,i) > armJointsLimits(2,2))
            disp(['WARNING: Arm joint 2 is violating its position limits at waypoint ',num2str(i)]);
        end
        if(x(18,i) < armJointsLimits(3,1) || x(18,i) > armJointsLimits(3,2))
            disp(['WARNING: Arm joint 3 is violating its position limits at waypoint ',num2str(i)]);
        end
        if(x(19,i) < armJointsLimits(4,1) || x(19,i) > armJointsLimits(4,2))
            disp(['WARNING: Arm joint 4 is violating its position limits at waypoint ',num2str(i)]);
        end
        if(x(20,i) < armJointsLimits(5,1) || x(20,i) > armJointsLimits(5,2))
            disp(['WARNING: Arm joint 5 is violating its position limits at waypoint ',num2str(i)]);
        end
        if(x(21,i) < armJointsLimits(6,1) || x(21,i) > armJointsLimits(6,2))
            disp(['WARNING: Arm joint 6 is violating its position limits at waypoint ',num2str(i)]);
        end
    end

    toc
    iu = cumsum(abs(u(1,:)));
    disp(['Total speed applied joint 1: ',num2str(iu(end)),' rad/s'])
    iu = cumsum(abs(u(2,:)));
    disp(['Total speed applied joint 2: ',num2str(iu(end)),' rad/s'])
    iu = cumsum(abs(u(3,:)));
    disp(['Total speed applied joint 3: ',num2str(iu(end)),' rad/s'])
    iu = cumsum(abs(u(4,:)));
    disp(['Total speed applied joint 4: ',num2str(iu(end)),' rad/s'])
    iu = cumsum(abs(u(5,:)));
    disp(['Total speed applied joint 5: ',num2str(iu(end)),' rad/s'])
    iu = cumsum(abs(u(6,:)));
    disp(['Total speed applied joint 6: ',num2str(iu(end)),' rad/s'])
    iu = cumsum(abs(u(7,:)));
    disp(['Total speed applied right wheels: ',num2str(iu(end)),' rad/s'])
    iu = cumsum(abs(u(8,:)));
    disp(['Total speed applied left wheels: ',num2str(iu(end)),' rad/s'])
    iu = cumsum(abs(u(9,:)));
    disp(['Total speed applied front steering joints: ',num2str(iu(end)),' rad/s'])
    iu = cumsum(abs(u(10,:)));
    disp(['Total speed applied back steering joints: ',num2str(iu(end)),' rad/s'])

    figure(1)
    % Plotting first arm config
    [TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(x(16:21,1));
    TWB = getTraslation([x(10,1),x(11,1),zBC])*getZRot(x(12,1));
    TW0 = TWB*TB0;
    TW1 = TWB*TB1;
    TW2 = TWB*TB2;
    TW3 = TWB*TB3;
    TW4 = TWB*TB4;
    TW5 = TWB*TB5;
    TW6 = TWB*TB6;
    plot3([TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4) TW4(1,4) TW5(1,4) TW6(1,4)],...
          [TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4) TW4(2,4) TW5(2,4) TW6(2,4)],...
          [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4) TW4(3,4) TW5(3,4) TW6(3,4)], 'Color', [0.75, 0.75, 0.75], 'LineWidth', 2.5);
    hold on;
    
    % Plotting last arm config
    [TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(x(16:21,end-1));
    TWB = getTraslation([x(10,end-1),x(11,end-1),zBC])*getZRot(x(12,end-1));
    TW0 = TWB*TB0;
    TW1 = TWB*TB1;
    TW2 = TWB*TB2;
    TW3 = TWB*TB3;
    TW4 = TWB*TB4;
    TW5 = TWB*TB5;
    TW6 = TWB*TB6;
    plot3([TW0(1,4) TW1(1,4) TW2(1,4) TW3(1,4) TW4(1,4) TW5(1,4) TW6(1,4)],...
          [TW0(2,4) TW1(2,4) TW2(2,4) TW3(2,4) TW4(2,4) TW5(2,4) TW6(2,4)],...
          [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4) TW4(3,4) TW5(3,4) TW6(3,4)], 'Color', [0.75, 0.75, 0.75], 'LineWidth', 2.5);
      
    % Plotting first rover position
    TWB = getTraslation([x(10,1),x(11,1),zBC])*getZRot(x(12,1));
    TB1 = getTraslation([dfx,dfy,-zBC]);
    TB2 = getTraslation([-dfx,dfy,-zBC]);
    TB3 = getTraslation([-dfx,-dfy,-zBC]);
    TB4 = getTraslation([dfx,-dfy,-zBC]);
    TW1 = TWB*TB1;
    TW2 = TWB*TB2;
    TW3 = TWB*TB3;
    TW4 = TWB*TB4;
    plot3([TWB(1,4) TW1(1,4) TWB(1,4) TW2(1,4) TWB(1,4) TW3(1,4) TWB(1,4) TW4(1,4)],...
          [TWB(2,4) TW1(2,4) TWB(2,4) TW2(2,4) TWB(2,4) TW3(2,4) TWB(2,4) TW4(2,4)],...
          [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', [0.75, 0.75, 0.75], 'LineWidth', 2.5);
   
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
              [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', [0.75, 0.75, 0.75], 'LineWidth', 2.5);

    quiver3(TWB(1,4), TWB(2,4), TWB(3,4), cos(x(12,end-1))/2, sin(x(12,end-1))/2, 0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.7)
    quiver3(TWB(1,4), TWB(2,4), TWB(3,4), -sin(x(12,end-1))/2, cos(x(12,end-1))/2, 0, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.7)
    quiver3(TWB(1,4), TWB(2,4), TWB(3,4), 0, 0, 1/2, 'Color', 'c', 'LineWidth', 2, 'MaxHeadSize', 0.7)
      
    % Plotting scenario
    daspect([1 1 1])
    contourf(X,Y,dilatedObstMap+obstMap);
    plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 5, 'Color', 'y')
    plot3(x(10,:),x(11,:),zBC*ones(size(x,2)), 'LineWidth', 5, 'Color', [1,0.5,0])
    title('Mobile manipulator trajectories', 'interpreter', ...
    'latex','fontsize',18)
    plot3(x0(10,1:end-1),x0(11,1:end-1), zBC*ones(size(t,2)-1,2), 'LineWidth', 5, 'Color', [0,0,0.6])
    plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', 'c')
    plot3(referencePath(1,:), referencePath(2,:), zBC*ones(size(referencePath,2)), 'LineWidth', 5, 'Color', [0.1,0.5,0.1])

    hold off;


%     figure(2)
%     plot(t,x(16:21,:))
%     title('Evolution of the arm joints', 'interpreter', ...
%     'latex','fontsize',18)
%     legend('$\theta_1$','$\theta_2$',...
%            '$\theta_3$','$\theta_4$','$\theta_5$','$\theta_6$', 'interpreter', ...
%            'latex','fontsize',18)
%     xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
%     ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
%     grid
%     
%     figure(3)
%     plot(t,x(22:25,:))
%     title('Evolution of the steering joints', 'interpreter', ...
%     'latex','fontsize',18)
%     legend('$\theta_{s1}$','$\theta_{s2}$',...
%            '$\theta_{s3}$','$\theta_{s4}$', 'interpreter', ...
%            'latex','fontsize',18)
%     xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
%     ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
%     grid
%        
%     figure(4)
%     plot(t,u(1:6,:))
%     title('Actuating joints velocities','interpreter','latex')
%     xlabel('t(s)','interpreter','latex','fontsize',18)
%     ylabel('$\dot\theta(rad/s$)','interpreter','latex','fontsize',18)
%     legend('$\dot\theta_1$','$\dot\theta_2$',...
%            '$\dot\theta_3$','$\dot\theta_4$','$\dot\theta_5$','$\dot\theta_6$', 'interpreter', ...
%            'latex','fontsize',18)
%               
%     figure(5)
%     plot(t,u(7:8,:))
%     title('Actuating wheels speed','interpreter','latex')
%     xlabel('t(s)','interpreter','latex','fontsize',18)
%     ylabel('$\omega(rad/s$)','interpreter','latex','fontsize',18)
%     legend('$\omega_R$','$\omega_L$', 'interpreter', ...
%            'latex','fontsize',18)
%               
%     figure(6)
%     plot(t,u(9:10,:))
%     title('Actuating steering speed','interpreter','latex')
%     xlabel('t(s)','interpreter','latex','fontsize',18)
%     ylabel('$\omega(rad/s$)','interpreter','latex','fontsize',18)
%     legend('$\omega_F$','$\omega_B$', 'interpreter', ...
%            'latex','fontsize',18)


    % sim('simulink_sherpa_tt_simpleBase',15);

end


