addpath('../../../ARES-DyMu_matlab/Global Path Planning/functions')

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

safetyDistance = 0.7;
mapResolution = 0.05;
vehicleSpeed = 0.1;

armJointsLimits = [-360 +360;
                   -120  80;
                   -140 +140;
                   -360 +360;
                   -135 +135;
                   -360 +360]*pi/180;

% Constraints 
xB0 = 2;
yB0 = 2.5;
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

xef = 1.8;
yef = 8.9;
zef = 0.2;
rollef = 0;
pitchef = pi;
yawef = 0;

tf = 15;
dt = 0.5;
t = 0:dt:tf;

fc = 1000000;
oc = 0.11;
bc = 2;
ac = 60;
lc = 0.5;

maxIter = 1000;

% FMM to compute totalCostMap
load('obstMap2','obstMap')
dilatedObstMap = dilateObstMap(obstMap, safetyDistance, mapResolution);
distMap = mapResolution*bwdist(dilatedObstMap);
costMap = 1./distMap;
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
[gBCMx, gBCMy] = calculateGradient(mapResolution*totalCostMap);

tau = 0.5;
[referencePath,~,~,~] = getPathGDM(totalCostMap,iInit,iGoal,tau);
referencePath = (referencePath-1)*mapResolution;

% State vectors
x = zeros(26,size(t,2));
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
x(12,1) = zB0;
% WTBspeed
x(13,1) = 0;
x(14,1) = 0;
% ArmJoints
x(15,1) = 0;
x(16,1) = -pi/2;
x(17,1) = pi/2;
x(18,1) = 0;
x(19,1) = pi/2;
x(20,1) = 0;
% ArmJoints Speed
x(21,1) = 0;
x(22,1) = 0;
x(23,1) = 0;
x(24,1) = 0;
x(25,1) = 0;
x(26,1) = 0;

% Initial control law
u = zeros(8,size(t,2));

% Target state and control trajectories
x0 = zeros(20,size(t,2));
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

u0 = zeros(8,size(t,2));

Jac = zeros(6,6,size(t,2));
heading = zeros(1,size(t,2));

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
        Jac(:,:,i-1) = jacobian(x(15:20,i-1));
        heading(i-1) = atan2(x(14,i-1),x(13,i-1));
        x(1,i) = cos(heading(i-1))*x(4,i-1) - sin(heading(i-1))*x(5,i-1) + x(10,i-1);
        x(2,i) = sin(heading(i-1))*x(4,i-1) + cos(heading(i-1))*x(5,i-1) + x(11,i-1);
        x(3,i) = x(6,i-1) + x(12,i-1);
        x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*x(21:26,i-1)*dt; 
        x(10:11,i) = x(10:11,i-1) + x(13:14,i-1)*dt;    
        x(12,i) = x(12,i-1); 
        x(13:14,i) = x(13:14,i-1) + u(1:2,i-1)*dt; 
        x(15:20,i) = x(15:20,i-1) + x(21:26,i-1)*dt;
        x(21:26,i) = x(21:26,i-1) + u(3:8,i-1)*dt; 
    end
    Jac(:,:,end) = jacobian(x(15:20,end));
    heading(end) = atan2(x(14,end),x(13,end));

    xh0 = x0 - x;
    uh0 = u0 - u;    
    
    % Quadratize cost function along the trajectory
    Q = zeros(size(x,1),size(x,1),size(t,2));
     
    Qend = zeros(size(x,1),size(x,1));
    Qend(1,1) = fc;
    Qend(2,2) = fc;
    Qend(3,3) = fc;
    Qend(7,7) = fc;
    Qend(8,8) = fc;
    Qend(9,9) = fc;
    Qend(13,13) = fc;
    Qend(14,14) = fc;
    
    R = eye(size(u,1));
    R(1,1) = bc;
    R(2,2) = bc;
    R(3,3) = ac;
    R(4,4) = ac;
    R(5,5) = ac;
    R(6,6) = ac;
    R(7,7) = ac;
    R(8,8) = ac;                       
    
    % Linearize the system dynamics and constraints along the trajectory    
    A = zeros(size(x,1),size(x,1),size(t,2));
    for i = 1:size(t,2)
        A(1,4,i) = cos(heading(i));
        A(1,5,i) = -sin(heading(i));
        A(1,10,i) = 1;
        A(2,4,i) = sin(heading(i));
        A(2,5,i) = cos(heading(i));
        A(2,11,i) = 1;
        A(3,6,i) = 1;
        A(3,12,i) = 1;
        A(4,4,i) = 1;
        A(5,5,i) = 1;
        A(6,6,i) = 1;
        A(7,7,i) = 1;
        A(8,8,i) = 1;
        A(9,9,i) = 1;
        A(4:9,15:20,i) = Jac(:,:,i);
        A(10,10,i) = 1;
        A(10,13,i) = dt;
        A(11,11,i) = 1;
        A(11,14,i) = dt;
        A(12,12,i) = 1;
        A(13,13,i) = 1;
        A(14,14,i) = 1;
        A(15,15,i) = 1;
        A(15,21,i) = dt;
        A(16,16,i) = 1;
        A(16,22,i) = dt;
        A(17,17,i) = 1;
        A(17,23,i) = dt;
        A(18,18,i) = 1;
        A(18,24,i) = dt;
        A(19,19,i) = 1;
        A(19,25,i) = dt;
        A(20,20,i) = 1;
        A(20,26,i) = dt;
        A(21,21,i) = 1;
        A(22,22,i) = 1;
        A(23,23,i) = 1;
        A(24,24,i) = 1;
        A(25,25,i) = 1;
        A(26,26,i) = 1;
    end
    
    B = zeros(size(x,1),size(u,1),size(t,2));
    for i = 1:size(t,2)
        B(13,1,i) = dt;
        B(14,2,i) = dt;
        B(21,3,i) = dt;
        B(22,4,i) = dt;
        B(23,5,i) = dt;
        B(24,6,i) = dt;
        B(25,7,i) = dt;
        B(26,8,i) = dt;
    end    
    
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
    
    % Logaritmic sharpness
    tLog = 10 + 0.01*iter;

    
    % Total cost map cost
    Ox = zeros(size(Q,1),size(t,2)); 
    for i = 1:size(t,2)-1
        [Ox(13,i), Ox(14,i)] = getGradientTotalCost(x(10,i), x(11,i), mapResolution, gBCMx, gBCMy);
        Ox(13,i) = oc*Ox(13,i);
        Ox(14,i) = oc*Ox(14,i);
    end
    
    % Joints limits cost
    Jux = zeros(size(Q,1),size(t,2));
    for i = 1:size(t,2)-1
        Jux(15:20,i) = lc.*getGradientLogBarrierCost(x(15:20,i),armJointsLimits(:,2),tLog,0);
    end
    
    Jdx = zeros(size(Q,1),size(t,2));
    for i = 1:size(t,2)-1
        Jdx(15:20,i) = lc.*getGradientLogBarrierCost(x(15:20,i),armJointsLimits(:,1),tLog,1);
    end

    
    P(:,:,end) = Qend;
    s(:,:,end) = -Qend*xh0(:,end) + Ox(:,end) + Jux(:,end) + Jdx(:,end);
    
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
            A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*B(:,:,i)*uh0(:,i) - Q(:,:,i)*xh0(:,i) + Ox(:,i) + Jux(:,i) + Jdx(:,i);
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
    if norm(uh)<0.0001*norm(u) || ((norm(uh)<0.08*norm(u))&&(endDist<0.05))
        disp(['SLQ found the optimal control input within ',num2str(iter-1),' iterations'])
        break;
    else
        % Line search to optimize alfa
        alfa = 1:-0.1:0.0001;
        J = zeros(size(alfa));
        uk = u;
        for n = 1:size(alfa,2)
            u = uk + alfa(n)*uh;
            for i = 2:size(t,2)
                Jac(:,:,i-1) = jacobian(x(15:20,i-1));
                heading(i-1) = atan2(x(14,i-1),x(13,i-1));
                x(1,i) = cos(heading(i-1))*x(4,i-1) - sin(heading(i-1))*x(5,i-1) + x(10,i-1);
                x(2,i) = sin(heading(i-1))*x(4,i-1) + cos(heading(i-1))*x(5,i-1) + x(11,i-1);
                x(3,i) = x(6,i-1) + x(12,i-1);
                x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*x(21:26,i-1)*dt; 
                x(10:11,i) = x(10:11,i-1) + x(13:14,i-1)*dt;    
                x(12,i) = x(12,i-1); 
                x(13:14,i) = x(13:14,i-1) + u(1:2,i-1)*dt; 
                x(15:20,i) = x(15:20,i-1) + x(21:26,i-1)*dt;
                x(21:26,i) = x(21:26,i-1) + u(3:8,i-1)*dt; 
            end
            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Qend*(x(:,end)-x0(:,end))...
                + oc*getTotalCost(x(10,end), x(11,end), mapResolution, totalCostMap)...
                + lc*sum(getSimpleLogBarrierCost(x(15:20,end),armJointsLimits(:,2),tLog,0))...
                + lc*sum(getSimpleLogBarrierCost(x(15:20,end),armJointsLimits(:,2),tLog,1));
            for i = 1:size(t,2)-1
                J(n) = J(n) + 1/2*((x(:,i)-x0(:,i)).'*Q(:,:,i)*(x(:,i)-x0(:,i))...
                    + (u(:,i)-u0(:,i)).'*R*(u(:,i)-u0(:,i)))...
                    + oc*getTotalCost(x(10,i), x(11,i), mapResolution, totalCostMap)...
                    + lc*sum(getSimpleLogBarrierCost(x(15:20,i),armJointsLimits(:,2),tLog,0))...
                    + lc*sum(getSimpleLogBarrierCost(x(15:20,i),armJointsLimits(:,2),tLog,1));
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
    [TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(x(15:20,1));
    TWB = getTraslation([x(10,1),x(11,1),x(12,1)])*getZRot(heading(1));
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
    [TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(x(15:20,end-1));
    TWB = getTraslation([x(10,end-1),x(11,end-1),x(12,end-1)])*getZRot(heading(end-1));
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
    daspect([1 1 1])
    contourf(X,Y,dilatedObstMap+obstMap);
    plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 5)
    plot3(x(10,1:end-1),x(11,1:end-1),x(12,1:end-1), 'LineWidth', 5)
    title('Mobile manipulator trajectories', 'interpreter', ...
    'latex','fontsize',18)
    hold off;

    disp(['Iteration number ',num2str(iter-1)])

end

for i = 2:size(t,2)
    Jac(:,:,i-1) = jacobian(x(15:20,i-1));
    heading(i-1) = atan2(x(14,i-1),x(13,i-1));
    x(1,i) = cos(heading(i-1))*x(4,i-1) - sin(heading(i-1))*x(5,i-1) + x(10,i-1);
    x(2,i) = sin(heading(i-1))*x(4,i-1) + cos(heading(i-1))*x(5,i-1) + x(11,i-1);
    x(3,i) = x(6,i-1) + x(12,i-1);
    x(4:9,i) = x(4:9,i-1) + Jac(:,:,i-1)*x(21:26,i-1)*dt; 
    x(10:11,i) = x(10:11,i-1) + x(13:14,i-1)*dt;    
    x(12,i) = x(12,i-1); 
    x(13:14,i) = x(13:14,i-1) + u(1:2,i-1)*dt; 
    x(15:20,i) = x(15:20,i-1) + x(21:26,i-1)*dt;
    x(21:26,i) = x(21:26,i-1) + u(3:8,i-1)*dt; 
end

toc
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

figure(1)
hold off;
[TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(x(15:20,1));
TWB = getTraslation([x(10,1),x(11,1),x(12,1)])*getZRot(heading(1));
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
[TB0, TB1, TB2, TB3, TB4, TB5, TB6] = direct(x(15:20,end-1));
TWB = getTraslation([x(10,end-1),x(11,end-1),x(12,end-1)])*getZRot(heading(end-1));
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
daspect([1 1 1])
contourf(X,Y,dilatedObstMap+obstMap);
map = [0 0.6   0
   0.6 0.3 0
   0.6 0   0];
colormap(map);
plot3(x(1,:),x(2,:),x(3,:), 'LineWidth', 5)
plot3(x(10,1:end-1),x(11,1:end-1),x(12,1:end-1), 'LineWidth', 5)
plot3(referencePath(:,1),referencePath(:,2), zB0*ones(size(referencePath,1),2), 'LineWidth', 5)
title('Mobile manipulator trajectories', 'interpreter', ...
'latex','fontsize',18) 
hold off



figure(2)
plot(t,x(13:14,:))
hold on
plot(t,x(21:26,:))
title('Evolution of the state', 'interpreter', ...
'latex','fontsize',18)
legend('$x_{B}^.$','$y_{B}^.$','$\tau_1^.$','$\tau_2^.$','$\tau_3^.$','$\tau_4^.$','$\tau_5^.$','$\tau_6^.$', 'interpreter', ...
'latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
%ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
grid
hold off

% figure(3)
% plot(t,u)
% title('Actuating acc (u)','interpreter','latex')
% xlabel('t(s)','interpreter','latex')
% ylabel('$a(m/s^2$)','interpreter','latex')
% legend('x displacement','y displacement','$Joint 1$','$Joint 2$',...
%        '$Joint 3$','$Joint 4$','$Joint 5$','$Joint 6$', 'interpreter', ...
%        'latex','fontsize',18)

figure(4)
plot(t,x(15:20,:))
title('Manipulator Joints','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('$q(rad)$','interpreter','latex')
legend('$\theta_1$','$\theta_2$',...
       '$\theta_3$','$\theta_4$','$\theta_5$','$\theta_6$', 'interpreter', ...
       'latex','fontsize',18)
hold off
% figure(4)
% contourf(gBCMx)
% figure(2)
% contourf(gBCMy)
% sim('simulink_sherpa_tt_simpleBase',15);


