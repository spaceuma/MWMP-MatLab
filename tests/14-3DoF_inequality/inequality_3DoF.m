%% Initialization

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

% Minimum step actuation percentage
lineSearchStep = 0.30; 

% Max acceptable dist
distThreshold = 0.005;

% Maximum number of iterations
maxIter = 10000;

% Percentage of constrained timesteps for resampling
resamplingThreshold = 30;

% Activate/deactivate dynamic plotting during the simulation
dynamicPlotting = 0;

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
numPureStateConstraints = 2;
J0 = zeros(numPureStateConstraints,timeSteps);
J = J0;
G = zeros(numPureStateConstraints,numStates,timeSteps);
h = zeros(numPureStateConstraints,timeSteps);

% % The pure state constraints are defined as:
% % G*x + h <= 0
% 
G(1,9,:) = 1;
h(1,:) = -1.8;

G(2,9,:) = -1;
h(2,:) = -1.8;


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
        
    % Update reference trajectories    
    xs = zeros(numStates,timeSteps);
    us = zeros(numInputs,timeSteps);
    
    % Define the sequential state and input vectors
    xs0 = zeros(numStates,timeSteps);
    us0 = zeros(numInputs,timeSteps);

    for i = 1:timeSteps
        xs0(:,i) = Q(:,:,i)*(x(:,i) - x0(:,i));
        us0(:,i) = R(:,:,i)*(u(:,i) - u0(:,i));
        
        % This is the official Sideris way, but the reference vector is not
        % fed back
%         xs0(:,i) = Q(:,:,i)*x(:,i) + x0(:,i);
%         us0(:,i) = R(:,:,i)*u(:,i) + u0(:,i);
    end
    
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
            
    % Active Constraints definition
    % State input constraints
    tl = [];
    p = cell(timeSteps,1);
    paux = zeros(numStateInputConstraints,1);
    pl = zeros(numStateInputConstraints,1);
    
    for i = 1:timeSteps
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
    
    Cl = zeros(numActiveSIConstraints,numStates,timeSteps);
    Dl = zeros(numActiveSIConstraints,numInputs,timeSteps);
    rl = zeros(numActiveSIConstraints,timeSteps);
    
    for i = tl
        Cl(pl(p{i}),:,i) = C(p{i},:,i);
        Dl(pl(p{i}),:,i) = D(p{i},:,i);
%         rl(pl(p{i}),i) = r(p{i},i);
    end
    
    
    % Pure state constraints
    tk = [];
    q = cell(timeSteps,1);
    qaux = zeros(numPureStateConstraints,1);
    ql = zeros(numPureStateConstraints,1);

    for i = 1:timeSteps
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
    
    
    Gk = zeros(numActivePSConstraints,numStates,timeSteps);
    hk = zeros(numActivePSConstraints,timeSteps);
    
    for i = tk
        Gk(ql(q{i}),:,i) = G(q{i},:,i);
%         hk(ql(q{i}),i) = h(q{i},i);
    end
    
    % Predefinitions
    Dh = zeros(numActiveSIConstraints,numActiveSIConstraints,timeSteps);
    E = zeros(numActiveSIConstraints,numStates,timeSteps);
    rh = zeros(numActiveSIConstraints,timeSteps);
    
    Ah = zeros(numStates,numStates,timeSteps);
    Rh = zeros(numStates,numStates,timeSteps);
    Qh = zeros(numStates,numStates,timeSteps);
    x0h = zeros(numStates,timeSteps);
    u0h = zeros(numStates,timeSteps);
    
    for i = tl
        Dh(pl(p{i}),pl(p{i}),i) = inv(Dl(pl(p{i}),:,i)/R(:,:,i)*Dl(pl(p{i}),:,i).');
    end
    
    for i = 1:timeSteps

        E(:,:,i) = Cl(:,:,i) - Dl(:,:,i)/R(:,:,i)*K(:,:,i).';
        rh(:,i) = rl(:,i) - Dl(:,:,i)/R(:,:,i)*us0(:,i);
        
        Ah(:,:,i) = A(:,:,i) - B(:,:,i)/R(:,:,i)*(K(:,:,i).' + Dl(:,:,i).'*Dh(:,:,i)*E(:,:,i));
        Rh(:,:,i) = B(:,:,i)/R(:,:,i)*(eye(numInputs,numInputs)-Dl(:,:,i).'*Dh(:,:,i)*Dl(:,:,i)/R(:,:,i))*B(:,:,i).';
        Qh(:,:,i) = Q(:,:,i) - K(:,:,i)/R(:,:,i)*K(:,:,i).' + E(:,:,i).'*Dh(:,:,i)*E(:,:,i);
        x0h(:,i) = xs0(:,i) - K(:,:,i)/R(:,:,i)*us0(:,i) + E(:,:,i).'*Dh(:,:,i)*rh(:,i);
        u0h(:,i) = -B(:,:,i)/R(:,:,i)*(us0(:,i) + Dl(:,:,i).'*Dh(:,:,i)*rh(:,i));
    end
        
    % LQ problem solution
    M = zeros(numStates,numStates,timeSteps);
    P = zeros(numStates,numStates,timeSteps);
    z = zeros(numStates,timeSteps);
    P(:,:,end) = Q(:,:,end);
    z(:,end) = xs0(:,end);
    
    % Solve backward
    for i = timeSteps-1:-1:1
        M(:,:,i) = inv(eye(numStates) + Rh(:,:,i)*P(:,:,i+1));
        P(:,:,i) = Qh(:,:,i) + Ah(:,:,i).'*P(:,:,i+1)*M(:,:,i)*Ah(:,:,i);
        z(:,i) = Ah(:,:,i).'*M(:,:,i).'*z(:,i+1) + ...
            Ah(:,:,i).'*P(:,:,i+1)*M(:,:,i)*u0h(:,i) + x0h(:,i);
    end
    
    Gamma = zeros(numActivePSConstraints*size(tk,2),numStates);
    Gammak = zeros(numActivePSConstraints,numStates,timeSteps, size(tk,2));
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
        warning('Cannot compute inverse, determinant is 0');
        correctIndex = [];        
        for k = 1:size(tk,2)
            correctIndex = [correctIndex (k-1)*numActivePSConstraints+ql(q{tk(k)}).'];
        end
        Faux = F(correctIndex,correctIndex);
        if (rank(Faux) < size(Faux,2))
            warning('Cannot compute inverse, linearly dependant terms in matrix');
            [Ffraux, indRemoved, indKeeped, indSimilar] = getFullRankMatrix(Faux);
            nuV(correctIndex(indKeeped)) = Ffraux\...
                (-Gamma(correctIndex(indKeeped),:)*xs(:,1) - ...
                y(correctIndex(indKeeped)) - H(correctIndex(indKeeped)));
            
            nuV(correctIndex(indRemoved)) = nuV(correctIndex(indSimilar));

        else
            invF = zeros(size(F));
            invFaux = inv(Faux);
            invF(correctIndex,correctIndex) = invFaux; 
            nuV(:) = invF*(-Gamma*xs(:,1) - y - H);
        end
    elseif (rank(F) < numActivePSConstraints*size(tk,2))
        warning('Cannot compute inverse, linearly dependant terms in matrix');
        [Ffr, indRemoved, indKeeped, indSimilar] = getFullRankMatrix(F);
        nuV(indKeeped) = Ffr\(-Gamma(indKeeped,:)*xs(:,1) - y(indKeeped) - H(indKeeped));
        nuV(indRemoved) = nuV(indSimilar);
    else
        nuV(:) = F\(-Gamma*xs(:,1) - y - H);
    end
    
    nu = zeros(numActivePSConstraints,timeSteps);
    for i = 1:size(tk,2)
        nu(:,tk(i)) = nuV((i-1)*numActivePSConstraints+1:i*numActivePSConstraints);
    end   
    
    s = zeros(numStates,timeSteps);
    if numActivePSConstraints
        for i = 1:timeSteps
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
        
    v = zeros(numStates,timeSteps);
    lambda = zeros(numStates,timeSteps);
    mu = zeros(numActiveSIConstraints,timeSteps);

    % Solve forward
    for i = 1:timeSteps-1
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
        rhoi = ones(numStateInputConstraints,timeSteps);
        deltai = ones(numStateInputConstraints,timeSteps);
        
        rhoj = ones(numPureStateConstraints,timeSteps);
        deltaj = ones(numPureStateConstraints,timeSteps);

        if numStateInputConstraints||numPureStateConstraints
            for n = 1:timeSteps
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
        end
              
        thetak = min(-rhoi(~I & deltai>0)./deltai(~I & deltai>0));
        betak = min(-rhoj(~J & deltaj>0)./deltaj(~J & deltaj>0));

        alfak = min([1 thetak betak]);      
        
        if alfak == 1
            step3 = true;
            
            % Line search to optimize alfa
            alfa = 1:-lineSearchStep:0.0001;
            Jcost = zeros(size(alfa));

            xk = x;
            uk = u;
            for n = 1:size(alfa,2)
%                 x = xk + alfa(n)*xs;
                u = uk + alfa(n)*us;
                
                x = forwardIntegrateSystem(x, u, dt);

                Jcost(n) = 1/2*(x(:,end)-x0(:,end)).'*Q(:,:,end)*(x(:,end)-x0(:,end));
                for i = 1:timeSteps-1
                    Jcost(n) = Jcost(n) + 1/2*((x(:,i)-x0(:,i)).'*Q(:,:,i)*(x(:,i)-x0(:,i))...
                        + (u(:,i)-u0(:,i)).'*R(:,:,i)*(u(:,i)-u0(:,i)));
                end            
            end
            [mincost, ind] = min(Jcost);
            alfamin = alfa(ind);

            % Update controller
%             x = xk + alfamin*xs;
            u = uk + alfamin*us;
            
            x = forwardIntegrateSystem(x, u, dt);

            
        else
            step3 = false;
            
            % Update controller
%             x = x + alfak*xs;
            u = u + alfak*us;
            
            x = forwardIntegrateSystem(x, u, dt);
            
            for i = 1:numStateInputConstraints
                for n = 1:timeSteps
                    if(-rhoi(i,n)/deltai(i,n) == alfak && ~I(i,n) && n < timeSteps)
                        I(i,n) = 1;
                    end
                end
                if sum(I(i,:)) > timeSteps*resamplingThreshold/100
                    I(i,:) = 0;
                    disp('Resampling...');
                end
            end
            for j = 1:numPureStateConstraints
                for n = 1:timeSteps
                    if(-rhoj(j,n)/deltaj(j,n) == alfak && ~J(j,n)&& n > 1)                        
                        J(j,n) = 1;
                    end
                end
                if sum(J(j,:)) > timeSteps*resamplingThreshold/100
                    J(j,:) = 0;
                    disp('Resampling...');
                end
            end
            
        end
    end
    
    % Exit condition
    endDist = norm(x(1:3,end)-x0(1:3,end));
    
    if step3        
        % Step 3
        if size(tl,2) > 0
            minMu = zeros(1,timeSteps);
            iS = zeros(1,timeSteps);

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
            minNu = zeros(1,timeSteps);
            jS = zeros(1,timeSteps);

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
          (norm(us)<0.001*norm(u) ||...
          ((norm(us)<0.025*norm(u))&&(endDist<distThreshold)))

            disp(['SLQ found the optimal control input within ',num2str(iter-1),' iterations'])            
            
%             u = u + us;            
%             x = forwardIntegrateSystem(x, u, dt);


            break;
        else
            if minimumMu <= minimumNu && size(p{mS},1) > 0
                I(p{mS}(iS(mS)),mS) = 0;
            elseif minimumMu > minimumNu && size(q{lS},1) > 0
                J(q{lS}(jS(lS)),lS) = 0;
            end
        end
    end
    
    disp(['Iteration number ',num2str(iter-1),...
          ', distance to goal = ',num2str(endDist)]);
    
    if iter > maxIter
        cprintf('err','MP-FB SLQ failed to generate a motion plan\n')
        if endDist > distThreshold
            error('The goal was not reachable');
        elseif norm(us)>0.001*norm(u)
            error('The SLQR algorithm failed to converge');
        elseif minimumMu <= -1e-5 || minimumNu <=-1e-5
            error('SLQR could not keep to the imposed constraints');
        else
            error('Something unexpected prevented SLQR to converge');
        end
    end
              
end

%% Plots
    
x = forwardIntegrateSystem(x, u, dt);

toc
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
sim('manipulator3DoF',t(end));

