addpath('../../../ARES-DyMu_matlab/Global Path Planning/functions')
addpath('../../maps')
addpath('../../models')
addpath('../../costs')
 
clear

tic
% System properties
global dfx;
dfx = 1;
global dfy;
dfy = 1;
global r;
r = 0.2;

safetyDistance = 0.7;
mapResolution = 0.05;
vehicleSpeed = 0.1;

% Constraints 
xB0 = 2;
yB0 = 2.5;
yawB0 = -pi/2;

xBf = 8;
yBf = 8.5;
yawBf = -pi/4;

tf = 15;
dt = 0.2;
t = 0:dt:tf;

fc = 1000000; % Final state cost, 1000000
foc = 0; % Final orientation cost, 1000000
tc = 0.11; % Total cost map cost, 0.11
bc = 0.1; % Base actuation cost, 2
dc = 0.2; % Steering cost, 2
sm = 5; % Influence of turns into final speed, tune till convergence


maxIter = 1000;

% FMM to compute totalCostMap
load('obstMap2','obstMap')
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
iGoal = [round(xBf/mapResolution)+1 round(yBf/mapResolution)+1];

[totalCostMap, ~] = computeTmap(costMap,iGoal);

% Quadratizing totalCostMap
totalCostMap(totalCostMap == Inf) = NaN;
[gTCMx, gTCMy] = calculateGradient(mapResolution*totalCostMap);

tau = 0.5;
[referencePath,~,~,~] = getPathGDM(totalCostMap,iInit,iGoal,tau);
referencePath = (referencePath-1)*mapResolution;

% State vectors
x = zeros(10,size(t,2));
% WTB
x(1,1) = xB0;
x(2,1) = yB0;
x(3,1) = yawB0;
% Bspeed
x(4,1) = 0;
x(5,1) = 0;
x(6,1) = 0;
% Steering joints
x(7,1) = 0;
x(8,1) = 0;
x(9,1) = 0;
x(10,1) = 0;

% Initial control law
u = zeros(4,size(t,2));

% Target state and control trajectories
x0 = zeros(10,size(t,2));
x0(1,end) = xBf;
x0(2,end) = yBf;
x0(3,end) = yawBf;
x0(4,end) = 0;
x0(5,end) = 0;
x0(6,end) = 0;
x0(7,end) = 0;
x0(8,end) = 0;
x0(9,end) = 0;
x0(10,end) = 0;

u0 = zeros(4,size(t,2));

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
        x(1,i) = x(1,i-1) + cos(x(3,i-1))*x(4,i-1)*dt - sin(x(3,i-1))*x(5,i-1)*dt;
        x(2,i) = x(2,i-1) + sin(x(3,i-1))*x(4,i-1)*dt + cos(x(3,i-1))*x(5,i-1)*dt;
        x(3,i) = x(3,i-1) + x(6,i-1)*dt;
        x(4,i) = - 1/2*(sin(x(7,i-1))*u(1,i-1)+sin(x(9,i-1))*u(2,i-1));
        x(5,i) = r/2*(cos(x(7,i-1))*u(1,i-1) + cos(x(9,i-1))*u(2,i-1));
        x(6,i) = 2*r/dfx*(cos(x(7,i-1))*u(1,i-1) - cos(x(9,i-1))*u(2,i-1));
        x(7:8,i) =  x(7:8,i-1) + u(3,i-1)*dt;
        x(9:10,i) =  x(9:10,i-1) + u(4,i-1)*dt;        
    end

    xh0 = x0 - x;
    uh0 = u0 - u;    
    
    % Quadratize cost function along the trajectory
    Q = zeros(size(x,1),size(x,1),size(t,2));
     
    Qend = zeros(size(x,1),size(x,1));
    Qend(1,1) = fc;
    Qend(2,2) = fc;
    Qend(3,3) = foc;
    
    R = eye(size(u,1));
    R(1,1) = bc;
    R(2,2) = bc;
    R(3,3) = dc;
    R(4,4) = dc;                    
    
    % Linearize the system dynamics and constraints along the trajectory    
    A = zeros(size(x,1),size(x,1),size(t,2));
    A(:,:,1) = [1 0 0 0 0 0  0 0 0 0;
                0 1 0 0 0 0  0 0 0 0;
                0 0 1 0 0 dt 0 0 0 0;
                0 0 0 0 0 0  0 0 0 0;
                0 0 0 0 0 0  0 0 0 0;
                0 0 0 0 0 0  0 0 0 0;
                0 0 0 0 0 0  1 0 0 0;
                0 0 0 0 0 0  0 1 0 0;
                0 0 0 0 0 0  0 0 1 0;
                0 0 0 0 0 0  0 0 0 1];
    
    A(1,3,1) = dt*(-sin(x(3,1))*x(4,1)/sm-cos(x(3,1))*x(5,1)/sm);
    A(1,4,1) = dt*(cos(x(3,1))+sin(x(3,1))*x(3,1)/sm);
    A(1,5,1) = -dt*(sin(x(3,1))-cos(x(3,1))*x(3,1)/sm);
    
    A(2,3,1) = dt*(cos(x(3,1))*x(4,1)/sm-sin(x(3,1))*x(5,1)/sm);
    A(2,4,1) = dt*(sin(x(3,1))-cos(x(3,1))*x(3,1)/sm);
    A(2,5,1) = dt*(cos(x(3,1))+sin(x(3,1))*x(3,1)/sm);
    
    
            
    for i = 2:size(t,2)
        A(:,:,i) = [1 0 0 0 0 0  0 0 0 0;
                    0 1 0 0 0 0  0 0 0 0;
                    0 0 1 0 0 dt 0 0 0 0;
                    0 0 0 0 0 0  0 0 0 0;
                    0 0 0 0 0 0  0 0 0 0;
                    0 0 0 0 0 0  0 0 0 0;
                    0 0 0 0 0 0  1 0 0 0;
                    0 0 0 0 0 0  0 1 0 0;
                    0 0 0 0 0 0  0 0 1 0;
                    0 0 0 0 0 0  0 0 0 1];
                
                    A(1,3,i) = dt*(-sin(x(3,i-1))*x(4,i-1)/sm-cos(x(3,i-1))*x(5,i-1)/sm);
                    A(1,4,i) = dt*(cos(x(3,i-1))+sin(x(3,i-1))*x(3,i-1)/sm);
                    A(1,5,i) = -dt*(sin(x(3,i-1))-cos(x(3,i-1))*x(3,i-1)/sm);

                    A(2,3,i) = dt*(cos(x(3,i-1))*x(4,i-1)/sm-sin(x(3,i-1))*x(5,i-1)/sm);
                    A(2,4,i) = dt*(sin(x(3,i-1))-cos(x(3,i-1))*x(3,i-1)/sm);
                    A(2,5,i) = dt*(cos(x(3,i-1))+sin(x(3,i-1))*x(3,i-1)/sm);
    end
    
    B = zeros(size(x,1),size(u,1),size(t,2));
    B(:,:,1) = [0                    0                   0    0;
                0                    0                   0    0;
                0                    0                   0    0;
                -sin(x(7,1))/2      -sin(x(9,1))/2       0    0;
                cos(x(7,1))*r/2      cos(x(9,1))*r/2     0    0;
                cos(x(7,1))*2*r/dfy -cos(x(9,1))*2*r/dfy 0    0;
                0                    0                   dt   0;
                0                    0                   dt   0;
                0                    0                   0    dt;
                0                    0                   0    dt];
    for i = 2:size(t,2)
        B(:,:,i) = [0                      0                     0    0;
                    0                      0                     0    0;
                    0                      0                     0    0;
                    -sin(x(7,i-1))/2      -sin(x(9,i-1))/2       0    0;
                    cos(x(7,i-1))*r/2      cos(x(9,i-1))*r/2     0    0;
                    cos(x(7,i-1))*2*r/dfy -cos(x(9,i-1))*2*r/dfy 0    0;
                    0                      0                     dt   0;
                    0                      0                     dt   0;
                    0                      0                     0    dt;
                    0                      0                     0    dt];
    end    
    
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
    
    % Logaritmic sharpness
    tLog = 10 + 0.01*iter;
    
    % Total cost map cost
%     Tcmx = zeros(size(Q,1),size(t,2)); 
%     for i = 1:size(t,2)-1
%         [Tcmx(13,i), Tcmx(14,i)] = getGradientTotalCost(x(10,i), x(11,i), mapResolution, gTCMx, gTCMy);
%         Tcmx(13,i) = tc*Tcmx(13,i);
%         Tcmx(14,i) = tc*Tcmx(14,i);
%     end
    
    P(:,:,end) = Qend;
    s(:,:,end) = -Qend*xh0(:,end) ;%+ Tcmx(:,end);
    
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
            A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*B(:,:,i)*uh0(:,i) - Q(:,:,i)*xh0(:,i);%...
            %+ Tcmx(:,i);
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
                x(1,i) = x(1,i-1) + cos(x(3,i-1))*x(4,i-1)*dt - sin(x(3,i-1))*x(5,i-1)*dt;
                x(2,i) = x(2,i-1) + sin(x(3,i-1))*x(4,i-1)*dt + cos(x(3,i-1))*x(5,i-1)*dt;
                x(3,i) = x(3,i-1) + x(6,i-1)*dt;
                x(4,i) = - 1/2*(sin(x(7,i-1))*u(1,i-1)+sin(x(9,i-1))*u(2,i-1));
                x(5,i) = r/2*(cos(x(7,i-1))*u(1,i-1) + cos(x(9,i-1))*u(2,i-1));
                x(6,i) = 2*r/dfx*(cos(x(7,i-1))*u(1,i-1) - cos(x(9,i-1))*u(2,i-1));
                x(7:8,i) =  x(7:8,i-1) + u(3,i-1)*dt;
                x(9:10,i) =  x(9:10,i-1) + u(4,i-1)*dt; 
            end
            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Qend*(x(:,end)-x0(:,end));%...
%                 + tc*getTotalCost(x(10,end), x(11,end), mapResolution, totalCostMap)...
%                 + lc*sum(getSimpleLogBarrierCost(x(15:20,end),armJointsLimits(:,2),tLog,0))...
%                 + lc*sum(getSimpleLogBarrierCost(x(15:20,end),armJointsLimits(:,2),tLog,1))...
%                 + oc*getTotalCost(x(10,end), x(11,end), mapResolution, obstLogCostMap);
            for i = 1:size(t,2)-1
                J(n) = J(n) + 1/2*((x(:,i)-x0(:,i)).'*Q(:,:,i)*(x(:,i)-x0(:,i))...
                    + (u(:,i)-u0(:,i)).'*R*(u(:,i)-u0(:,i)));%...
%                     + tc*getTotalCost(x(10,i), x(11,i), mapResolution, totalCostMap)...
%                     + lc*sum(getSimpleLogBarrierCost(x(15:20,i),armJointsLimits(:,2),tLog,0))...
%                     + lc*sum(getSimpleLogBarrierCost(x(15:20,i),armJointsLimits(:,2),tLog,1))...
%                     + oc*getTotalCost(x(10,i), x(11,i), mapResolution, obstLogCostMap);
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
    TWB = getTraslation([x(1,1),x(2,1),0.645])*getZRot(x(3,1));
    TB1 = getTraslation([dfx,dfy,-0.645]);
    TB2 = getTraslation([-dfx,dfy,-0.645]);
    TB3 = getTraslation([-dfx,-dfy,-0.645]);
    TB4 = getTraslation([dfx,-dfy,-0.645]);
    TW1 = TWB*TB1;
    TW2 = TWB*TB2;
    TW3 = TWB*TB3;
    TW4 = TWB*TB4;
    plot3([TWB(1,4) TW1(1,4) TWB(1,4) TW2(1,4) TWB(1,4) TW3(1,4) TWB(1,4) TW4(1,4)],...
          [TWB(2,4) TW1(2,4) TWB(2,4) TW2(2,4) TWB(2,4) TW3(2,4) TWB(2,4) TW4(2,4)],...
          [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', 'r', 'LineWidth', 2.5);
    hold on;
    TWB = getTraslation([x(1,end),x(2,end),0.645])*getZRot(x(3,end));
    TW1 = TWB*TB1;
    TW2 = TWB*TB2;
    TW3 = TWB*TB3;
    TW4 = TWB*TB4;
    plot3([TWB(1,4) TW1(1,4) TWB(1,4) TW2(1,4) TWB(1,4) TW3(1,4) TWB(1,4) TW4(1,4)],...
              [TWB(2,4) TW1(2,4) TWB(2,4) TW2(2,4) TWB(2,4) TW3(2,4) TWB(2,4) TW4(2,4)],...
              [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', 'r', 'LineWidth', 2.5);
    daspect([1 1 1])
    contourf(X,Y,dilatedObstMap+obstMap);
    plot3(x(1,:),x(2,:),0.645*ones(size(x(1,:))), 'LineWidth', 5)
    title('Mobile robot trajectories', 'interpreter', ...
    'latex','fontsize',18)
    plot3(xBf,yBf,0.645*ones(size(x(1,:))), 'MarkerSize', 20, 'Marker', '.', 'Color', 'c')

    hold off;

%     disp(['Iteration number ',num2str(iter-1)])

end

for i = 2:size(t,2)
    x(1,i) = x(1,i-1) + cos(x(3,i-1))*x(4,i-1)*dt - sin(x(3,i-1))*x(5,i-1)*dt;
    x(2,i) = x(2,i-1) + sin(x(3,i-1))*x(4,i-1)*dt + cos(x(3,i-1))*x(5,i-1)*dt;
    x(3,i) = x(3,i-1) + x(6,i-1)*dt;
    x(4,i) = - 1/2*(sin(x(7,i-1))*u(1,i-1)+sin(x(9,i-1))*u(2,i-1));
    x(5,i) = r/2*(cos(x(7,i-1))*u(1,i-1) + cos(x(9,i-1))*u(2,i-1));
    x(6,i) = 2*r/dfx*(cos(x(7,i-1))*u(1,i-1) - cos(x(9,i-1))*u(2,i-1));
    x(7:8,i) =  x(7:8,i-1) + u(3,i-1)*dt;
    x(9:10,i) =  x(9:10,i-1) + u(4,i-1)*dt; 
end

toc
% iu = cumsum(abs(u(1,:)));
% disp(['Total acc applied x displacement: ',num2str(iu(end)),' m/s^2'])
% iu = cumsum(abs(u(2,:)));
% disp(['Total acc applied y displacement: ',num2str(iu(end)),' m/s^2'])
% iu = cumsum(abs(u(3,:)));
% disp(['Total acc applied joint 1: ',num2str(iu(end)),' m/s^2'])
% iu = cumsum(abs(u(4,:)));
% disp(['Total acc applied joint 2: ',num2str(iu(end)),' m/s^2'])
% iu = cumsum(abs(u(5,:)));
% disp(['Total acc applied joint 3: ',num2str(iu(end)),' m/s^2'])
% iu = cumsum(abs(u(6,:)));
% disp(['Total acc applied joint 4: ',num2str(iu(end)),' m/s^2'])
% iu = cumsum(abs(u(7,:)));
% disp(['Total acc applied joint 5: ',num2str(iu(end)),' m/s^2'])
% iu = cumsum(abs(u(8,:)));
% disp(['Total acc applied joint 6: ',num2str(iu(end)),' m/s^2'])

figure(1)
TWB = getTraslation([x(1,1),x(2,1),0.645])*getZRot(x(3,1));
TB1 = getTraslation([dfx,dfy,-0.645]);
TB2 = getTraslation([-dfx,dfy,-0.645]);
TB3 = getTraslation([-dfx,-dfy,-0.645]);
TB4 = getTraslation([dfx,-dfy,-0.645]);
TW1 = TWB*TB1;
TW2 = TWB*TB2;
TW3 = TWB*TB3;
TW4 = TWB*TB4;
plot3([TWB(1,4) TW1(1,4) TWB(1,4) TW2(1,4) TWB(1,4) TW3(1,4) TWB(1,4) TW4(1,4)],...
          [TWB(2,4) TW1(2,4) TWB(2,4) TW2(2,4) TWB(2,4) TW3(2,4) TWB(2,4) TW4(2,4)],...
          [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', 'r', 'LineWidth', 2.5);
hold on;
TWB = getTraslation([x(1,end),x(2,end),0.645])*getZRot(x(3,end));
TW1 = TWB*TB1;
TW2 = TWB*TB2;
TW3 = TWB*TB3;
TW4 = TWB*TB4;
plot3([TWB(1,4) TW1(1,4) TWB(1,4) TW2(1,4) TWB(1,4) TW3(1,4) TWB(1,4) TW4(1,4)],...
          [TWB(2,4) TW1(2,4) TWB(2,4) TW2(2,4) TWB(2,4) TW3(2,4) TWB(2,4) TW4(2,4)],...
          [TWB(3,4) TW1(3,4) TWB(3,4) TW2(3,4) TWB(3,4) TW3(3,4) TWB(3,4) TW4(3,4)], 'Color', 'r', 'LineWidth', 2.5);
daspect([1 1 1])
contourf(X,Y,dilatedObstMap+obstMap);
map = [0 0.6   0
   0.6 0.3 0
   0.6 0   0];
colormap(map);
plot3(x(1,:),x(2,:),0.645*ones(size(x(1,:))), 'LineWidth', 5)
title('Mobile robot trajectories', 'interpreter', ...
'latex','fontsize',18) 
plot3(xBf,yBf,0.645*ones(size(x(1,:))), 'MarkerSize', 20, 'Marker', '.', 'Color', 'c')

hold off



figure(2)
plot(t,x(4:6,:))
title('Evolution of the state (A)', 'interpreter', ...
'latex','fontsize',18)
legend('$x_{c}^.$','$y_{c}^.$','$\omega_c$', 'interpreter', ...
'latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
%ylabel('$\theta^. (rad)$', 'interpreter', 'latex','fontsize',18)
grid
hold off

figure(3)
plot(t,x(7:10,:))
title('Evolution of the state (B)', 'interpreter', ...
'latex','fontsize',18)
legend('$\theta_{d1}$','$\theta_{d2}$','$\theta_{d3}$','$\theta_{d4}$',...
'interpreter','latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
%ylabel('$\theta^. (rad)$', 'interpreter', 'latex','fontsize',18)
grid
hold off

figure(4)
plot(t,u)
title('Actuating control input (u)','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('$\omega(rad/s$)','interpreter','latex')
legend('$\omega_R$','$\omega_L$','$\omega_sf$','$\omega_sb$',...
       'interpreter','latex','fontsize',18)

% figure(4)
% plot(t,x(15:20,:))
% title('Manipulator Joints','interpreter','latex')
% xlabel('t(s)','interpreter','latex')
% ylabel('$q(rad)$','interpreter','latex')
% legend('$\theta_1$','$\theta_2$',...
%        '$\theta_3$','$\theta_4$','$\theta_5$','$\theta_6$', 'interpreter', ...
%        'latex','fontsize',18)
% hold off

% sim('simulink_sherpa_tt_simpleBase',15);


