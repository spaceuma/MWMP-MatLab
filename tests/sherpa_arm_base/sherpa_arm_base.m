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

safetyDistance = 1;
mapResolution = 0.05;
vehicleSpeed = 0.1;

armJointsLimits = [-360 +360;
                   -120  80;
                   -140 +140;
                   -360 +360;
                   -135 +135;
                   -360 +360]*pi/180;

% State costs
fc = 1000000; % Final state cost, 1000000
foc = 1000000; % Final orientation cost, 0
rtc = 1; % Reference path cost

% Input costs
bc = 5; % Base actuation cost, 2
sc = 20; % Steering cost, 2
ac = 1000; % Arm actuation cost, 60

% Extra costs
sm = 50; % Influence of diff turns into final speed, tune till convergence
sm2 = 9999999999999; % Influence of steer turns into final speed, tune till convergence
lc = 1; % Joints limits cost, 0.5
oc = 0.0; % Obstacles limits cost, 0.0
tc = 0.0; % Total cost map cost, 0.11

lineSearchStep = 0.01;

iterFCApproaching = 0;

maxIter = 500;

% Constraints 
xB0 = 2;
yB0 = 2.5;
zB0 = zBC;
yawB0 = 0;

rollei = 0;
pitchei = pi;
yawei = 0;

xef = 7.6;
yef = 6.6;
zef = 0.2;
rollef = 0;
pitchef = pi;
yawef = 0;

tf = 60;
dt = 0.8;
t = 0:dt:tf;

TWB = getTraslation([xB0,yB0,zB0])*getZRot(yawB0);
[~, ~, ~, ~, ~, ~, TB6] = direct([0, -pi/2, pi/2, 0, pi/2, 0]);

TW6 = TWB*TB6;

xei = TW6(1,4);
yei = TW6(2,4);
zei = TW6(3,4);

% FMM to compute totalCostMap
load('obstMap1','obstMap')
dilatedObstMap = dilateObstMap(obstMap, safetyDistance, mapResolution);
distRiskMap = mapResolution*bwdist(dilatedObstMap);
costMap = 1./distRiskMap;
costMap = costMap./min(min(costMap));

distMap = mapResolution*bwdist(obstMap);
auxMap = 1./distMap;
gradient = 1;
minCost = max(max(costMap(costMap~=Inf)));
costMap(dilatedObstMap==1)= minCost + gradient*auxMap((dilatedObstMap==1))./min(min(auxMap((dilatedObstMap==1))));
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

% Generating obst log cost map
tLog = 10;
obstLogCostMap = zeros(size(distMap));
for i = 1:size(distMap,1)
    for j = 1:size(distMap,2)
        obstLogCostMap(j,i) = getSimpleLogBarrierCost(distMap(j,i),safetyDistance,tLog,1);
    end
end

[gOLCMx, gOLCMy] = calculateMapGradient(obstLogCostMap);

% State vectors
x = zeros(31,size(t,2));
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
x(16,1) = 0;
x(17,1) = -pi/2;
x(18,1) = pi/2;
x(19,1) = 0;
x(20,1) = pi/2;
x(21,1) = 0;
% ArmJoints Speed
x(22,1) = 0;
x(23,1) = 0;
x(24,1) = 0;
x(25,1) = 0;
x(26,1) = 0;
x(27,1) = 0;
% Steering joints
x(28,1) = 0;
x(29,1) = 0;
x(30,1) = 0;
x(31,1) = 0;

% Initial control law
u = zeros(10,size(t,2));

% Target state and control trajectories
x0 = zeros(31,size(t,2));

% Resize path
x1 = 1:size(referencePath,1);
x2 = linspace(1,size(referencePath,1),size(x,2));
resizedPath = interp1(x1,referencePath,x2);

x0(10,:) = resizedPath(:,1);
x0(11,:) = resizedPath(:,2);

x0(1,end) = xef;
x0(2,end) = yef;
x0(3,end) = zef;
x0(4,end) = 0;
x0(5,end) = 0;
x0(6,end) = 0;
x0(7,end) = rollef;
x0(8,end) = pitchef;
x0(9,end) = yawef;
x0(10,end) = 0;
x0(11,end) = 0;
x0(12,end) = 0;
x0(13,end) = 0;
x0(14,end) = 0;
x0(15,end) = 0;
x0(16,end) = 0;
x0(17,end) = 0;
x0(18,end) = 0;
x0(19,end) = 0;
x0(20,end) = 0;
x0(21,end) = 0;
x0(22,end) = 0;
x0(23,end) = 0;
x0(24,end) = 0;
x0(25,end) = 0;
x0(26,end) = 0;
x0(27,end) = 0;
x0(28,end) = 0;
x0(29,end) = 0;
x0(30,end) = 0;
x0(31,end) = 0;

u0 = zeros(10,size(t,2));

Jac = zeros(6,6,size(t,2));

% Plotting stuff
map = [0 0.6   0
       0.6 0.3 0
       0.6 0   0];
colormap(map);
xVect = linspace(0,9.95,200);
[X,Y] = meshgrid(xVect,xVect);

% SLQR algorithm
iter = 1;
while 1   
    % Forward integrate system equations
    for i = 2:size(t,2)
        Jac(:,:,i-1) = jacobian(x(16:21,i-1));
        % W2EE
        x(1,i) = cos(x(12,i-1))*x(4,i-1) + sin(x(12,i-1))*x(5,i-1) + x(10,i-1);
        x(2,i) = - sin(x(12,i-1))*x(4,i-1) + cos(x(12,i-1))*x(5,i-1) + x(11,i-1);
        x(3,i) = x(6,i-1) + zBC;
        % B2EE
        x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*x(22:27,i-1)*dt; 
        % W2B
        x(10,i) = x(10,i-1) + cos(x(12,i-1))*x(13,i-1)*dt + sin(x(12,i-1))*x(14,i-1)*dt;
        x(11,i) = x(11,i-1) - sin(x(12,i-1))*x(13,i-1)*dt + cos(x(12,i-1))*x(14,i-1)*dt;
        x(12,i) = x(12,i-1) + x(15,i-1)*dt;
        % Bspeed
        x(13,i) = r/2*(cos(x(28,i-1))*u(7,i-1) + cos(x(30,i-1))*u(8,i-1));
        x(14,i) = - r/2*(sin(x(28,i-1))*u(7,i-1) + sin(x(30,i-1))*u(8,i-1));
        x(15,i) = 2*r/dfx*(cos(x(28,i-1))*u(7,i-1) - cos(x(30,i-1))*u(8,i-1));
        % Arm Joints Position
        x(16:21,i) = x(16:21,i-1) + x(22:27,i-1)*dt;
        % Arm Joints Speed
        x(22:27,i) = x(22:27,i-1) + u(1:6,i-1)*dt;
        % Steering Joints Position
        x(28:29,i) =  x(28:29,i-1) + u(9,i-1)*dt;
        x(30:31,i) =  x(30:31,i-1) + u(10,i-1)*dt;
    end
    Jac(:,:,end) = jacobian(x(16:21,end));

    xh0 = x0 - x;
    uh0 = u0 - u;    
    
    % Quadratize cost function along the trajectory
    Q = zeros(size(x,1),size(x,1),size(t,2));
    Q(10,10,:) = rtc;
    Q(11,11,:) = rtc;
     
    Qend = zeros(size(x,1),size(x,1));
    Qend(1,1) = fc;
    Qend(2,2) = fc;
    Qend(3,3) = fc;
    Qend(7,7) = foc;
    Qend(8,8) = foc;
    Qend(9,9) = foc;
%     Qend(13,13) = fc;
%     Qend(14,14) = fc;
    
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
    A(1,5,1) = sin(x(12,1));
    A(1,10,1) = 1;

    % W2EEy
    A(2,4,1) = -sin(x(12,1));
    A(2,5,1) = cos(x(12,1));
    A(2,11,1) = 1;

    % W2EEz
    A(3,6,1) = 1;

    % B2EE
    A(4:9,4:9,1) = eye(6,6);
    A(4:9,22:27,1) = dt*Jac(:,:,1);

    % W2Bx
    A(10,10,1) = 1;
    A(10,12,1) = dt*(-sin(x(12,1))*x(13,1)/sm+cos(x(12,1))*x(14,1)/sm);
    A(10,13,1) = dt*(cos(x(12,1))+sin(x(12,1))*x(12,1)/sm);
    A(10,14,1) = dt*(sin(x(12,1))-cos(x(12,1))*x(12,1)/sm);

    % W2By
    A(11,11,1) = 1;
    A(11,12,1) = dt*(-cos(x(12,1))*x(13,1)/sm-sin(x(12,1))*x(14,1)/sm);
    A(11,13,1) = dt*(-sin(x(12,1))+cos(x(12,1))*x(12,1)/sm);
    A(11,14,1) = dt*(cos(x(12,1))+sin(x(12,1))*x(12,1)/sm);

    % W2B Heading
    A(12,12,1) = 1;
    A(12,15,1) = dt;

    % W2B Speed x
    A(13,28,1) = r/2*(-sin(x(28,1))*u(7,1))/sm2;
    A(13,30,1) = r/2*(-sin(x(30,1))*u(8,1))/sm2;
    
    % W2B Speed y
    A(14,28,1) = -r/2*cos(x(28,1))*u(7,1)/sm2;
    A(14,30,1) = -r/2*cos(x(30,1))*u(8,1)/sm2;
    
    % W2B Speed Heading
    A(15,28,1) = 2*r/dfx *(-sin(x(28,1))*u(7,1))/sm2;
    A(15,30,1) = -2*r/dfx*(-sin(x(30,1))*u(8,1))/sm2;
    
    % Arm Joints Position
    A(16:21,16:21,1) = eye(6,6);
    A(16:21,22:27,1) = dt*eye(6,6);
    
    % Arm Joints Speed
    A(22:27,22:27,1) = eye(6,6);

    % Steering Joints Position
    A(28:31,28:31,1) = eye(4,4);
    
    for i = 2:size(t,2)
        % W2EEx
        A(1,4,i) = cos(x(12,i-1));
        A(1,5,i) = sin(x(12,i-1));
        A(1,10,i) = 1;

        % W2EEy
        A(2,4,i) = -sin(x(12,i-1));
        A(2,5,i) = cos(x(12,i-1));
        A(2,11,i) = 1;

        % W2EEz
        A(3,6,i) = 1;

        % B2EE
        A(4:9,4:9,i) = eye(6,6);
        A(4:9,22:27,i) = dt*Jac(:,:,i-1);

        % W2Bx
        A(10,10,i) = 1;
        A(10,12,i) = dt*(-sin(x(12,i-1))*x(13,i-1)/sm+cos(x(12,i-1))*x(14,i-1)/sm);
        A(10,13,i) = dt*(cos(x(12,i-1))+sin(x(12,i-1))*x(12,i-1)/sm);
        A(10,14,i) = dt*(sin(x(12,i-1))-cos(x(12,i-1))*x(12,i-1)/sm);

        % W2By
        A(11,11,i) = 1;
        A(11,12,i) = dt*(-cos(x(12,i-1))*x(13,i-1)/sm-sin(x(12,i-1))*x(14,i-1)/sm);
        A(11,13,i) = dt*(-sin(x(12,i-1))+cos(x(12,i-1))*x(12,i-1)/sm);
        A(11,14,i) = dt*(cos(x(12,i-1))+sin(x(12,i-1))*x(12,i-1)/sm);

        % W2B Heading
        A(12,12,i) = 1;
        A(12,15,i) = dt;

        % W2B Speed x
        A(13,28,i) = r/2*(-sin(x(28,i-1))*u(7,i-1))/sm2;
        A(13,30,i) = r/2*(-sin(x(30,i-1))*u(8,i-1))/sm2;

        % W2B Speed y
        A(14,28,i) = -r/2*cos(x(28,i-1))*u(7,i-1)/sm2;
        A(14,30,i) = -r/2*cos(x(30,i-1))*u(8,i-1)/sm2;

        % W2B Speed Heading
        A(15,28,i) = 2*r/dfx *(-sin(x(28,i-1))*u(7,i-1))/sm2;
        A(15,30,i) = -2*r/dfx*(-sin(x(30,i-1))*u(8,i-1))/sm2;

        % Arm Joints Position
        A(16:21,16:21,i) = eye(6,6);
        A(16:21,22:27,i) = dt*eye(6,6);

        % Arm Joints Speed
        A(22:27,22:27,i) = eye(6,6);

        % Steering Joints Position
        A(28:31,28:31,i) = eye(4,4);
    end
    
    % Actuation (u) matrix
    B = zeros(size(x,1),size(u,1),size(t,2));
    
    % W2B Speed x
    B(13,7,1) = r/2*(cos(x(28,1)) + x(28,1)*sin(x(28,1))/sm2);
    B(13,8,1) = r/2*(cos(x(30,1)) + x(30,1)*sin(x(30,1))/sm2);
    
    % W2B Speed y
    B(14,7,1) = -r/2*(sin(x(28,1)) - x(28,1)*cos(x(28,1))/sm2);
    B(14,8,1) = -r/2*(sin(x(30,1)) - x(30,1)*cos(x(30,1))/sm2);
    
    % W2B Speed heading
    B(15,7,1) = 2*r/dfx*(cos(x(28,1)) + x(28,1)*sin(x(28,1))/sm2);
    B(15,8,1) = -2*r/dfx*(cos(x(30,1)) + x(30,1)*sin(x(30,1))/sm2);
    
    % Arm Joints Speed
    B(22:27,1:6,1) = dt*eye(6,6);
    
    % Steering Joints Position
    B(28:29,9,1) = dt;
    B(30:31,10,1) = dt;
    
    for i = 2:size(t,2)
        % W2B Speed x
        B(13,7,i) = r/2*(cos(x(28,i-1)) + x(28,i-1)*sin(x(28,i-1))/sm2);
        B(13,8,i) = r/2*(cos(x(30,i-1)) + x(30,i-1)*sin(x(30,i-1))/sm2);

        % W2B Speed y
        B(14,7,i) = -r/2*(sin(x(28,i-1)) - x(28,i-1)*cos(x(28,i-1))/sm2);
        B(14,8,i) = -r/2*(sin(x(30,i-1)) - x(30,i-1)*cos(x(30,i-1))/sm2);

        % W2B Speed heading
        B(15,7,i) = 2*r/dfx*(cos(x(28,i-1)) + x(28,1)*sin(x(28,i-1))/sm2);
        B(15,8,i) = -2*r/dfx*(cos(x(30,i-1)) + x(30,1)*sin(x(30,i-1))/sm2);

        % Arm Joints Speed
        B(22:27,1:6,i) = dt*eye(6,6);

        % Steering Joints Position
        B(28:29,9,i) = dt;
        B(30:31,10,i) = dt;
    end    
    
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
    
    % Logaritmic sharpness
    tLog = 10 + 0.01*iter;
    
    % Total cost map cost
    Tcmx = zeros(size(Q,1),size(t,2)); 
%     for i = 1:size(t,2)-1
%         [Tcmx(13,i), Tcmx(14,i)] = getGradientTotalCost(x(10,i), x(11,i), mapResolution, gTCMx, gTCMy);
%         Tcmx(13,i) = tc*Tcmx(13,i);
%         Tcmx(14,i) = tc*Tcmx(14,i);
%     end
    
    % Joints limits cost
    Jux = zeros(size(Q,1),size(t,2));
    for i = 1:size(t,2)-1
        Jux(15:20,i) = lc.*getGradientLogBarrierCost(x(15:20,i),armJointsLimits(:,2),tLog,0);
    end
    
    Jdx = zeros(size(Q,1),size(t,2));
    for i = 1:size(t,2)-1
        Jdx(15:20,i) = lc.*getGradientLogBarrierCost(x(15:20,i),armJointsLimits(:,1),tLog,1);
    end

    % Obstacles limits cost
    Ox = zeros(size(Q,1),size(t,2));
%     for i = 1:size(t,2)-1
%         [Ox(10,i), Ox(11,i)] = getGradientTotalCost(x(10,i), x(11,i), mapResolution, gOLCMx, gOLCMy);
%         Ox(10,i) = oc*Ox(10,i);
%         Ox(11,i) = oc*Ox(11,i);
%     end
    
    P(:,:,end) = Qend;
    s(:,:,end) = -Qend*xh0(:,end) + Tcmx(:,end) + Jux(:,end) + Jdx(:,end) + Ox(:,end);
    
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
            A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*B(:,:,i)*uh0(:,i) - Q(:,:,i)*xh0(:,i)...
            + Tcmx(:,i) + Jux(:,i) + Jdx(:,i) + Ox(:,i);
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
        (norm(uh)<0.0005*norm(u) || ((norm(uh)<0.2*norm(u))&&(endDist<0.025)))
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
                x(1,i) = cos(x(12,i-1))*x(4,i-1) + sin(x(12,i-1))*x(5,i-1) + x(10,i-1);
                x(2,i) = - sin(x(12,i-1))*x(4,i-1) + cos(x(12,i-1))*x(5,i-1) + x(11,i-1);
                x(3,i) = x(6,i-1) + zBC;
                % B2EE
                x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*x(22:27,i-1)*dt; 
                % W2B
                x(10,i) = x(10,i-1) + cos(x(12,i-1))*x(13,i-1)*dt + sin(x(12,i-1))*x(14,i-1)*dt;
                x(11,i) = x(11,i-1) - sin(x(12,i-1))*x(13,i-1)*dt + cos(x(12,i-1))*x(14,i-1)*dt;
                x(12,i) = x(12,i-1) + x(15,i-1)*dt;
                % W2Bspeed
                x(13,i) = r/2*(cos(x(28,i-1))*u(7,i-1) + cos(x(30,i-1))*u(8,i-1));
                x(14,i) = - r/2*(sin(x(28,i-1))*u(7,i-1) + sin(x(30,i-1))*u(8,i-1));
                x(15,i) = 2*r/dfx*(cos(x(28,i-1))*u(7,i-1) - cos(x(30,i-1))*u(8,i-1));
                % Arm Joints Position
                x(16:21,i) = x(16:21,i-1) + x(22:27,i-1)*dt;
                % Arm Joints Speed
                x(22:27,i) = x(22:27,i-1) + u(1:6,i-1)*dt;
                % Steering Joints Position
                x(28:29,i) =  x(28:29,i-1) + u(9,i-1)*dt;
                x(30:31,i) =  x(30:31,i-1) + u(10,i-1)*dt;
            end
            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Qend*(x(:,end)-x0(:,end))...
                + 100*~isSafePath(x(1,:),x(2,:),mapResolution,dilatedObstMap)...
                + tc*getTotalCost(x(10,end), x(11,end), mapResolution, totalCostMap)...
                + lc*sum(getSimpleLogBarrierCost(x(15:20,end),armJointsLimits(:,2),tLog,0))...
                + lc*sum(getSimpleLogBarrierCost(x(15:20,end),armJointsLimits(:,2),tLog,1))...
                + oc*getTotalCost(x(10,end), x(11,end), mapResolution, obstLogCostMap);
            for i = 1:size(t,2)-1
                J(n) = J(n) + 1/2*((x(:,i)-x0(:,i)).'*Q(:,:,i)*(x(:,i)-x0(:,i))...
                    + (u(:,i)-u0(:,i)).'*R*(u(:,i)-u0(:,i)))...
                    + tc*getTotalCost(x(10,i), x(11,i), mapResolution, totalCostMap)...
                    + lc*sum(getSimpleLogBarrierCost(x(15:20,i),armJointsLimits(:,2),tLog,0))...
                    + lc*sum(getSimpleLogBarrierCost(x(15:20,i),armJointsLimits(:,2),tLog,1))...
                    + oc*getTotalCost(x(10,i), x(11,i), mapResolution, obstLogCostMap);
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
          [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4) TW4(3,4) TW5(3,4) TW6(3,4)], 'Color', 'r', 'LineWidth', 2.5);
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
          [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4) TW4(3,4) TW5(3,4) TW6(3,4)], 'Color', 'r', 'LineWidth', 2.5);
      
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
    plot3(x(10,1:end-1),x(11,1:end-1),zBC*ones(size(x,2)-1), 'LineWidth', 5)
    title('Mobile manipulator trajectories', 'interpreter', ...
    'latex','fontsize',18)
    plot3(referencePath(:,1),referencePath(:,2), zBC*ones(size(referencePath,1),2), 'LineWidth', 5, 'Color', [1,0,1])
    plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', 'c')

    hold off;

    disp(['Iteration number ',num2str(iter-1), ', alpha = ', num2str(alfamin), ', endDist = ',num2str(endDist)])
    disp(['Is the path safe? ', num2str(isSafePath(x(10,:),x(11,:),mapResolution,dilatedObstMap))])
end

for i = 2:size(t,2)
    Jac(:,:,i-1) = jacobian(x(16:21,i-1));
    % W2EE
    x(1,i) = cos(x(12,i-1))*x(4,i-1) + sin(x(12,i-1))*x(5,i-1) + x(10,i-1);
    x(2,i) = - sin(x(12,i-1))*x(4,i-1) + cos(x(12,i-1))*x(5,i-1) + x(11,i-1);
    x(3,i) = x(6,i-1) + zBC;
    % B2EE
    x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*x(22:27,i-1)*dt; 
    % W2B
    x(10,i) = x(10,i-1) + cos(x(12,i-1))*x(13,i-1)*dt + sin(x(12,i-1))*x(14,i-1)*dt;
    x(11,i) = x(11,i-1) - sin(x(12,i-1))*x(13,i-1)*dt + cos(x(12,i-1))*x(14,i-1)*dt;
    x(12,i) = x(12,i-1) + x(15,i-1)*dt;
    % W2Bspeed
    x(13,i) = r/2*(cos(x(28,i-1))*u(7,i-1) + cos(x(30,i-1))*u(8,i-1));
    x(14,i) = - r/2*(sin(x(28,i-1))*u(7,i-1) + sin(x(30,i-1))*u(8,i-1));
    x(15,i) = 2*r/dfx*(cos(x(28,i-1))*u(7,i-1) - cos(x(30,i-1))*u(8,i-1));
    % Arm Joints Position
    x(16:21,i) = x(16:21,i-1) + x(22:27,i-1)*dt;
    % Arm Joints Speed
    x(22:27,i) = x(22:27,i-1) + u(1:6,i-1)*dt;
    % Steering Joints Position
    x(28:29,i) =  x(28:29,i-1) + u(9,i-1)*dt;
    x(30:31,i) =  x(30:31,i-1) + u(10,i-1)*dt;
    
    if(x(16,i) < armJointsLimits(1,1) || x(16,i) > armJointsLimits(1,2))
        disp('WARNING: Arm joint 1 is violating its position limits at waypoint ',num2str(i));
    end
    if(x(17,i) < armJointsLimits(2,1) || x(17,i) > armJointsLimits(2,2))
        disp('WARNING: Arm joint 2 is violating its position limits at waypoint ',num2str(i));
    end
    if(x(18,i) < armJointsLimits(3,1) || x(18,i) > armJointsLimits(3,2))
        disp('WARNING: Arm joint 3 is violating its position limits at waypoint ',num2str(i));
    end
    if(x(19,i) < armJointsLimits(4,1) || x(19,i) > armJointsLimits(4,2))
        disp('WARNING: Arm joint 4 is violating its position limits at waypoint ',num2str(i));
    end
    if(x(20,i) < armJointsLimits(5,1) || x(20,i) > armJointsLimits(5,2))
        disp('WARNING: Arm joint 5 is violating its position limits at waypoint ',num2str(i));
    end
    if(x(21,i) < armJointsLimits(6,1) || x(21,i) > armJointsLimits(6,2))
        disp('WARNING: Arm joint 6 is violating its position limits at waypoint ',num2str(i));
    end
end

toc
iu = cumsum(abs(u(1,:)));
disp(['Total acc applied joint 1: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(2,:)));
disp(['Total acc applied joint 2: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(3,:)));
disp(['Total acc applied joint 3: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(4,:)));
disp(['Total acc applied joint 4: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(5,:)));
disp(['Total acc applied joint 5: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(6,:)));
disp(['Total acc applied joint 6: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(7,:)));
disp(['Total speed applied right wheels: ',num2str(iu(end)),' m/s'])
iu = cumsum(abs(u(8,:)));
disp(['Total speed applied left wheels: ',num2str(iu(end)),' m/s'])
iu = cumsum(abs(u(9,:)));
disp(['Total speed applied front steering joints: ',num2str(iu(end)),' m/s'])
iu = cumsum(abs(u(10,:)));
disp(['Total speed applied back steering joints: ',num2str(iu(end)),' m/s'])

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
      [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4) TW4(3,4) TW5(3,4) TW6(3,4)], 'Color', 'r', 'LineWidth', 2.5);
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
      [TW0(3,4) TW1(3,4) TW2(3,4) TW3(3,4) TW4(3,4) TW5(3,4) TW6(3,4)], 'Color', 'r', 'LineWidth', 2.5);

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
plot3(x(10,1:end-1),x(11,1:end-1),zBC*ones(size(x,2)-1), 'LineWidth', 5)
title('Mobile manipulator trajectories', 'interpreter', ...
'latex','fontsize',18)
plot3(referencePath(:,1),referencePath(:,2), zBC*ones(size(referencePath,1),2), 'LineWidth', 5, 'Color', [1,0,1])
plot3(xef,yef,zef, 'MarkerSize', 20, 'Marker', '.', 'Color', 'c')

hold off;


% figure(2)
% plot(t,x(13:14,:))
% hold on
% plot(t,x(21:26,:))
% title('Evolution of the state', 'interpreter', ...
% 'latex','fontsize',18)
% legend('$x_{B}^.$','$y_{B}^.$','$\tau_1^.$','$\tau_2^.$','$\tau_3^.$','$\tau_4^.$','$\tau_5^.$','$\tau_6^.$', 'interpreter', ...
% 'latex','fontsize',18)
% xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
% %ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
% grid
% hold off

% figure(3)
% plot(t,u)
% title('Actuating acc (u)','interpreter','latex')
% xlabel('t(s)','interpreter','latex')
% ylabel('$a(m/s^2$)','interpreter','latex')
% legend('x displacement','y displacement','$Joint 1$','$Joint 2$',...
%        '$Joint 3$','$Joint 4$','$Joint 5$','$Joint 6$', 'interpreter', ...
%        'latex','fontsize',18)

% figure(4)
% plot(t,x(15:20,:))
% title('Manipulator Joints','interpreter','latex')
% xlabel('t(s)','interpreter','latex')
% ylabel('$q(rad)$','interpreter','latex')
% legend('$\theta_1$','$\theta_2$',...
%        '$\theta_3$','$\theta_4$','$\theta_5$','$\theta_6$', 'interpreter', ...
%        'latex','fontsize',18)
% hold off

% figure(4)
% contourf(gBCMx)
% figure(2)
% contourf(gBCMy)
% sim('simulink_sherpa_tt_simpleBase',15);


