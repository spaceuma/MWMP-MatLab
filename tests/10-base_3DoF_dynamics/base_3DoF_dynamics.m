%% Initialization

addpath('../../../ARES-DyMu_matlab/Global Path Planning/functions')
addpath('../../maps')
addpath('../../models')
addpath('../../models/3DoF')

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
global d4;
d4 = 0.695;


global dfx;
dfx = 0.7;
global dfy;
dfy = 0.7;
global r;
r = 0.2;
global zBC;
zBC = 0.645;

global reachabilityDistance;
reachabilityDistance = (a1+a2+d4-zBC-d0);

riskDistance = 1;
safetyDistance = 1.3;
mapResolution = 0.05;
vehicleSpeed = 0.1;

armJointsLimits = [-360 +360;
                   -120  80;
                   -140 +140]*pi/180;

%% Constraints 
xB0 = 2.0;
yB0 = 2.5;
zB0 = zBC;
yawB0 = 0;

qi = [0, -pi/2, pi/2];
rollei = 0;
pitchei = pi/2;
yawei = 0;

xef = 2.5;
yef = 8.7;
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

%% Costs
% State costs
fc = 1000000; % Final state cost, 1000000
foc = 0; % Final orientation cost, 0
fsc = 1000000; % Final zero speed cost, 1000000
rtc = 7; % Reference path cost 7

% Input costs
bc = 0.1; % Base actuation cost, 2
sc = 0.1; % Steering cost, 2
ac = 0.1; % Arm actuation cost, 60

% Extra costs
sm = 50; % Influence of diff turns into final speed, tune till convergence
sm2 = 99999999; % Influence of steer turns into final speed, tune till convergence

tf = 60;
dt = 0.1;
t = 0:dt:tf;

distThreshold = 0.031;

lineSearchStep = 0.05;

iterFCApproaching = 0;

maxIter = 500;

%% Algorithm
% FMM to compute totalCostMap
load('obstMap3','obstMap')
dilatedObstMap = dilateObstMap(obstMap, riskDistance, mapResolution);
safeObstMap = dilateObstMap(obstMap, safetyDistance, mapResolution);

% distRiskMap = mapResolution*bwdist(dilatedObstMap);
% costMap = 1./distRiskMap;
% costMap = costMap./min(min(costMap));

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

tau = 0.5;
[referencePath,~,~,~] = getPathGDM(totalCostMap,iInit,iGoal,tau);
referencePath = (referencePath-1)*mapResolution;
yaw = getYaw(referencePath, yawB0);
referencePath = [referencePath yaw];

% Generating obst log cost map
% tLog = 10;
% obstLogCostMap = zeros(size(distMap));
% for i = 1:size(distMap,1)
%     for j = 1:size(distMap,2)
%         obstLogCostMap(j,i) = getSimpleLogBarrierCost(distMap(j,i),safetyDistance,tLog,1);
%     end
% end
% 
% [gOLCMx, gOLCMy] = calculateMapGradient(obstLogCostMap);

% State vectors
x = zeros(22,size(t,2));
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
% ArmJoints
x(16,1) = qi(1);
x(17,1) = qi(2);
x(18,1) = qi(3);
% Steering joints
x(19,1) = 0;
x(20,1) = 0;
x(21,1) = 0;
x(22,1) = 0;

% Initial control law
u = zeros(7,size(t,2));

% Target state and control trajectories
x0 = zeros(22,size(t,2));

% Resize path
x1 = 1:size(referencePath,1);
x2 = linspace(1,size(referencePath,1),size(x,2));
resizedPath = interp1(x1,referencePath,x2);
reachabilityIndex = getDistanceIndex(resizedPath, [xef yef], reachabilityDistance);

x0(10,:) = resizedPath(:,1);
x0(11,:) = resizedPath(:,2);
x0(12,:) = resizedPath(:,3);


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
x0(10,end) = 0;
x0(11,end) = 0;
x0(12,end) = 0;
% Bspeed
x0(13,end) = 0;
x0(14,end) = 0;
x0(15,end) = 0;
% ArmJoints
x0(16,end) = 0;
x0(17,end) = 0;
x0(18,end) = 0;
% Steering joints
x0(19,end) = 0;
x0(20,end) = 0;
x0(21,end) = 0;
x0(22,end) = 0;

u0 = zeros(7,size(t,2));

Jac = zeros(6,3,size(t,2));

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
        Jac(:,:,i-1) = jacobian3(x(16:18,i-1));
        % W2EE
        x(1,i) = cos(x(12,i-1))*x(4,i-1) - sin(x(12,i-1))*x(5,i-1) + x(10,i-1);
        x(2,i) = sin(x(12,i-1))*x(4,i-1) + cos(x(12,i-1))*x(5,i-1) + x(11,i-1);
        x(3,i) = x(6,i-1) + zBC;
        % B2EE
        x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*u(1:3,i-1)*dt; 
        % W2B
        x(10,i) = x(10,i-1) + cos(x(12,i-1))*x(13,i-1)*dt - sin(x(12,i-1))*x(14,i-1)*dt;
        x(11,i) = x(11,i-1) + sin(x(12,i-1))*x(13,i-1)*dt + cos(x(12,i-1))*x(14,i-1)*dt;
        x(12,i) = x(12,i-1) + x(15,i-1)*dt;
        % Bspeed
        x(13,i) = r/2*(cos(x(19,i-1))*u(4,i-1) + cos(x(21,i-1))*u(5,i-1));
        x(14,i) = - r/2*(sin(x(19,i-1))*u(4,i-1) + sin(x(21,i-1))*u(5,i-1));
        x(15,i) = r/(2*dfx)*(cos(x(19,i-1))*u(4,i-1) - cos(x(21,i-1))*u(5,i-1));
        % Arm Joints Position
        x(16:18,i) = x(16:18,i-1) + u(1:3,i-1)*dt;
        % Steering Joints Position
        x(19:20,i) =  x(19:20,i-1) + u(6,i-1)*dt;
        x(21:22,i) =  x(21:22,i-1) + u(7,i-1)*dt;
    end
    Jac(:,:,end) = jacobian3(x(16:18,end));

    xh0 = x0 - x;
    uh0 = u0 - u;    
    
    % Quadratize cost function along the trajectory
    Q = zeros(size(x,1),size(x,1),size(t,2));
    Q(10,10,1:reachabilityIndex) = rtc;
    Q(11,11,1:reachabilityIndex) = rtc;
    Q(12,12,1:reachabilityIndex) = rtc;
    
    Q(10,10,reachabilityIndex:end) = rtc/9999999;
    Q(11,11,reachabilityIndex:end) = rtc/9999999;
    Q(12,12,reachabilityIndex:end) = rtc/9999999;
     
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
    
%     Qend(22,22) = fsc;
%     Qend(23,23) = fsc;
%     Qend(24,24) = fsc;
    
    R = eye(size(u,1));
    R(1,1) = ac;
    R(2,2) = ac;
    R(3,3) = ac;
    R(4,4) = bc;
    R(5,5) = bc;  
    R(6,6) = sc;
    R(7,7) = sc; 
    
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
    A(13,19,1) = r/2*(-sin(x(19,1))*u(4,1))/sm2;
    A(13,21,1) = r/2*(-sin(x(21,1))*u(5,1))/sm2;
    
    % W2B Speed y
    A(14,19,1) = -r/2*cos(x(19,1))*u(4,1)/sm2;
    A(14,21,1) = -r/2*cos(x(21,1))*u(5,1)/sm2;
    
    % W2B Speed Heading
    A(15,19,1) = r/(2*dfx) *(-sin(x(19,1))*u(4,1))/sm2;
    A(15,21,1) = -r/(2*dfx)*(-sin(x(21,1))*u(5,1))/sm2;
    
    % Arm Joints Position
    A(16:18,16:18,1) = eye(3,3);
    
    % Steering Joints Position
    A(19:22,19:22,1) = eye(4,4);
    
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
        A(13,19,i) = r/2*(-sin(x(19,i-1))*u(4,i-1))/sm2;
        A(13,21,i) = r/2*(-sin(x(21,i-1))*u(5,i-1))/sm2;

        % W2B Speed y
        A(14,19,i) = -r/2*cos(x(19,i-1))*u(4,i-1)/sm2;
        A(14,21,i) = -r/2*cos(x(21,i-1))*u(5,i-1)/sm2;

        % W2B Speed Heading
        A(15,19,i) = r/(2*dfx) *(-sin(x(19,i-1))*u(4,i-1))/sm2;
        A(15,21,i) = -r/(2*dfx)*(-sin(x(21,i-1))*u(5,i-1))/sm2;

        % Arm Joints Position
        A(16:18,16:18,i) = eye(3,3);

        % Steering Joints Position
        A(19:22,19:22,i) = eye(4,4);
    end
    
    % Actuation (u) matrix
    B = zeros(size(x,1),size(u,1),size(t,2));
    
    % BTEE
    B(4:9,1:3,1) = dt*Jac(:,:,1);

    % W2B Speed x
    B(13,4,1) = r/2*(cos(x(19,1)) + x(19,1)*sin(x(19,1))/sm2);
    B(13,5,1) = r/2*(cos(x(21,1)) + x(21,1)*sin(x(21,1))/sm2);
    
    % W2B Speed y
    B(14,4,1) = -r/2*(sin(x(19,1)) - x(19,1)*cos(x(19,1))/sm2);
    B(14,5,1) = -r/2*(sin(x(21,1)) - x(21,1)*cos(x(21,1))/sm2);
    
    % W2B Speed heading
    B(15,4,1) = r/(2*dfx)*(cos(x(19,1)) + x(19,1)*sin(x(19,1))/sm2);
    B(15,5,1) = -r/(2*dfx)*(cos(x(21,1)) + x(21,1)*sin(x(21,1))/sm2);
    
    % Arm Joints Position
    B(16:18,1:3,1) = dt*eye(3,3);
        
    % Steering Joints Position
    B(19:20,6,1) = dt;
    B(21:22,7,1) = dt;
    
    for i = 2:size(t,2)
        % BTEE
        B(4:9,1:3,i) = dt*Jac(:,:,i-1);

        % W2B Speed x
        B(13,4,i) = r/2*(cos(x(19,i-1)) + x(19,i-1)*sin(x(19,i-1))/sm2);
        B(13,5,i) = r/2*(cos(x(21,i-1)) + x(21,i-1)*sin(x(21,i-1))/sm2);

        % W2B Speed y
        B(14,4,i) = -r/2*(sin(x(19,i-1)) - x(19,i-1)*cos(x(19,i-1))/sm2);
        B(14,5,i) = -r/2*(sin(x(21,i-1)) - x(21,i-1)*cos(x(21,i-1))/sm2);

        % W2B Speed heading
        B(15,4,i) = r/(2*dfx)*(cos(x(19,i-1)) + x(19,i-1)*sin(x(19,i-1))/sm2);
        B(15,5,i) = -r/(2*dfx)*(cos(x(21,i-1)) + x(21,i-1)*sin(x(21,i-1))/sm2);

        % Arm Joints Position
        B(16:18,1:3,i) = dt*eye(3,3);

        % Steering Joints Position
        B(19:20,6,i) = dt;
        B(21:22,7,i) = dt;
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
                Jac(:,:,i-1) = jacobian3(x(16:18,i-1));
                % W2EE
                x(1,i) = cos(x(12,i-1))*x(4,i-1) - sin(x(12,i-1))*x(5,i-1) + x(10,i-1);
                x(2,i) = sin(x(12,i-1))*x(4,i-1) + cos(x(12,i-1))*x(5,i-1) + x(11,i-1);
                x(3,i) = x(6,i-1) + zBC;
                % B2EE
                x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*u(1:3,i-1)*dt; 
                % W2B
                x(10,i) = x(10,i-1) + cos(x(12,i-1))*x(13,i-1)*dt - sin(x(12,i-1))*x(14,i-1)*dt;
                x(11,i) = x(11,i-1) + sin(x(12,i-1))*x(13,i-1)*dt + cos(x(12,i-1))*x(14,i-1)*dt;
                x(12,i) = x(12,i-1) + x(15,i-1)*dt;
                % Bspeed
                x(13,i) = r/2*(cos(x(19,i-1))*u(4,i-1) + cos(x(21,i-1))*u(5,i-1));
                x(14,i) = - r/2*(sin(x(19,i-1))*u(4,i-1) + sin(x(21,i-1))*u(5,i-1));
                x(15,i) = r/(2*dfx)*(cos(x(19,i-1))*u(4,i-1) - cos(x(21,i-1))*u(5,i-1));
                % Arm Joints Position
                x(16:18,i) = x(16:18,i-1) + u(1:3,i-1)*dt;
                % Steering Joints Position
                x(19:20,i) =  x(19:20,i-1) + u(6,i-1)*dt;
                x(21:22,i) =  x(21:22,i-1) + u(7,i-1)*dt;
            end
            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Qend*(x(:,end)-x0(:,end))...
                + 100*~isSafePath(x(1,:),x(2,:),mapResolution,dilatedObstMap);
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
    plot3(referencePath(:,1),referencePath(:,2), zBC*ones(size(referencePath,1),2), 'LineWidth', 5, 'Color', [0,0,0.6])
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
        Jac(:,:,i-1) = jacobian3(x(16:18,i-1));
        % W2EE
        x(1,i) = cos(x(12,i-1))*x(4,i-1) - sin(x(12,i-1))*x(5,i-1) + x(10,i-1);
        x(2,i) = sin(x(12,i-1))*x(4,i-1) + cos(x(12,i-1))*x(5,i-1) + x(11,i-1);
        x(3,i) = x(6,i-1) + zBC;
        % B2EE
        x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*u(1:3,i-1)*dt; 
        % W2B
        x(10,i) = x(10,i-1) + cos(x(12,i-1))*x(13,i-1)*dt - sin(x(12,i-1))*x(14,i-1)*dt;
        x(11,i) = x(11,i-1) + sin(x(12,i-1))*x(13,i-1)*dt + cos(x(12,i-1))*x(14,i-1)*dt;
        x(12,i) = x(12,i-1) + x(15,i-1)*dt;
        % Bspeed
        x(13,i) = r/2*(cos(x(19,i-1))*u(4,i-1) + cos(x(21,i-1))*u(5,i-1));
        x(14,i) = - r/2*(sin(x(19,i-1))*u(4,i-1) + sin(x(21,i-1))*u(5,i-1));
        x(15,i) = r/(2*dfx)*(cos(x(19,i-1))*u(4,i-1) - cos(x(21,i-1))*u(5,i-1));
        % Arm Joints Position
        x(16:18,i) = x(16:18,i-1) + u(1:3,i-1)*dt;
        % Steering Joints Position
        x(19:20,i) =  x(19:20,i-1) + u(6,i-1)*dt;
        x(21:22,i) =  x(21:22,i-1) + u(7,i-1)*dt;

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

    toc
    iu = cumsum(abs(u(1,:)));
    disp(['Total speed applied joint 1: ',num2str(iu(end)),' rad/s'])
    iu = cumsum(abs(u(2,:)));
    disp(['Total speed applied joint 2: ',num2str(iu(end)),' rad/s'])
    iu = cumsum(abs(u(3,:)));
    disp(['Total speed applied joint 3: ',num2str(iu(end)),' rad/s'])    
    iu = cumsum(abs(u(4,:)));
    disp(['Total speed applied right wheels: ',num2str(iu(end)),' rad/s'])
    iu = cumsum(abs(u(5,:)));
    disp(['Total speed applied left wheels: ',num2str(iu(end)),' rad/s'])
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
    plot3(referencePath(:,1),referencePath(:,2), zBC*ones(size(referencePath,1),2), 'LineWidth', 5, 'Color', [0,0,0.6])
    plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', 'c')

    hold off;


    figure(2)
    plot(t,x(16:18,:))
    title('Evolution of the arm joints', 'interpreter', ...
    'latex','fontsize',18)
    legend('$\theta_1$','$\theta_2$','$\theta_3$', 'interpreter', ...
           'latex','fontsize',18)
    xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
    ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
    grid
    
    figure(3)
    plot(t,x(19:22,:))
    title('Evolution of the steering joints', 'interpreter', ...
    'latex','fontsize',18)
    legend('$\theta_{s1}$','$\theta_{s2}$',...
           '$\theta_{s3}$','$\theta_{s4}$', 'interpreter', ...
           'latex','fontsize',18)
    xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
    ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
    grid
       
    figure(4)
    plot(t,u(1:3,:))
    title('Actuating joints speed','interpreter','latex')
    xlabel('t(s)','interpreter','latex','fontsize',18)
    ylabel('$\dot\theta(m/s$)','interpreter','latex','fontsize',18)
    legend('$\dot\theta_1$','$\dot\theta_2$',...
           '$\dot\theta_3$','interpreter', ...
           'latex','fontsize',18)
              
    figure(5)
    plot(t,u(4:5,:))
    title('Actuating wheels speed','interpreter','latex')
    xlabel('t(s)','interpreter','latex','fontsize',18)
    ylabel('$\omega(m/s$)','interpreter','latex','fontsize',18)
    legend('$\omega_R$','$\omega_L$', 'interpreter', ...
           'latex','fontsize',18)
              
    figure(6)
    plot(t,u(6:7,:))
    title('Actuating steering speed','interpreter','latex')
    xlabel('t(s)','interpreter','latex','fontsize',18)
    ylabel('$\omega(m/s$)','interpreter','latex','fontsize',18)
    legend('$\omega_F$','$\omega_B$', 'interpreter', ...
           'latex','fontsize',18)

       
    %% Simulation
    mu_k = 0.1;
    mu_s = 1;
    vth = 0.001;
    
    k = 1e6;
    b = 1e4;
    
    sim('base_3DoF_dynamics_sim_forces',t(end));

end


