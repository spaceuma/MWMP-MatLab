clear

tic

%% Initialization
% System properties
% Arm links length
a1 = 0.20;
a2 = 0.15;
a3 = 0.10;

% Constraints
% Initial conditions
ti = 0;
xei = 0.2;
yei = 0;
thetai = 0;

% Goal
tf = 15;
xef = 0.1;
yef = 0.1;
thetaf = -pi/2;

% Time profile
dt = 0.05;
t = 0:dt:tf;

% State vector
q = zeros(3,size(t,2));
q(:,1) = inverso3([xei yei thetai],-1);

numStates = 6;
x = zeros(numStates,size(t,2));
% End effector position
x(1,1) = xei;
x(2,1) = yei;
% End effector orientation
x(3,1) = thetai;
% Arm joints speed
x(4,1) = 0;
x(5,1) = 0;
x(6,1) = 0;

% Initial control law
numInputs = 3;
u = zeros(numInputs,size(t,2)); % Arm joints acceleration

% Target state and control trajectories
x0 = zeros(numStates,size(t,2));
x0(1,end) = xef;
x0(2,end) = yef;
x0(3,end) = thetaf;
x0(4,end) = 0;
x0(5,end) = 0;
x0(6,end) = 0;

u0 = zeros(numInputs,size(t,2));

Jac = zeros(numInputs,numInputs,size(t,2));

%% SLQR algorithm
lineSearchStep = 0.2;
maxIter = 500; % Maximum number of iterations

iter = 1;
while 1   
    % Forward integrate system equations    
    for i = 2:size(t,2)
        % Arm joints position
        q(:,i) = q(:,i-1) + x(4:6,i-1)*dt;
        % Jacobian matrix
        Jac(:,:,i-1) = jacobiano3(q(:,i-1));
        % End effector pose
        x(1:3,i) = x(1:3,i-1) + Jac(:,:,i-1)*x(4:6,i-1)*dt;
        % Arm joints velocities
        x(4:6,i) = x(4:6,i-1) + u(:,i-1)*dt;       
    end
    Jac(:,:,end) = jacobiano3(q(:,end));
    
    % Update reference trajectories    
    xh0 = x0 - x;
    uh0 = u0 - u;  
    
    
    % Quadratize cost function along the trajectory    
    Q = zeros(numStates,numStates,size(t,2));
    for i = 1:size(t,2)
        Q(:,:,i) = [0 0 0 0 0 0;
                    0 0 0 0 0 0;
                    0 0 0 0 0 0;
                    0 0 0 0 0 0;
                    0 0 0 0 0 0;
                    0 0 0 0 0 0];
    end
     
    Qend = zeros(numStates,numStates);
    Qend(:,:) = [1000000 0 0 0 0 0;
                 0 1000000 0 0 0 0;
                 0 0 1000000 0 0 0;
                 0 0 0       1000000 0 0;
                 0 0 0       0 1000000 0;
                 0 0 0       0 0 1000000];
        
    R = [1 0 0;
         0 1 0;
         0 0 1];

    % Linearize the system dynamics and constraints along the trajectory    
    A = zeros(numStates,numStates,size(t,2));
    for i = 1:size(t,2)
        A(:,:,i) = [1 0 0 dt*Jac(1,1,i) dt*Jac(1,2,i) dt*Jac(1,3,i);
                    0 1 0 dt*Jac(2,1,i) dt*Jac(2,2,i) dt*Jac(2,3,i);
                    0 0 1 dt*Jac(3,1,i) dt*Jac(3,2,i) dt*Jac(3,3,i);
                    0 0 0 1        0        0;
                    0 0 0 0        1        0;
                    0 0 0 0        0        1];
    end
    
    B = zeros(numStates,numInputs,size(t,2));
    for i = 1:size(t,2)
        B(:,:,i) = [0  0  0;
                    0  0  0;
                    0  0  0;
                    dt 0  0;
                    0  dt 0;
                    0  0  dt];
    end    
    
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
    
    P(:,:,end) = Qend;
    s(:,:,end) = -Qend*xh0(:,end);
    
    xh = zeros(numStates,size(t,2));
    uh = zeros(numInputs,size(t,2));
    v = zeros(size(xh));
    lambdah = zeros(size(s));
       
    % Solve backward
    for i = size(t,2)-1:-1:1
        M(:,:,i) = inv(eye(size(B,1)) + B(:,:,i)/R*B(:,:,i).'*P(:,:,i+1));
        P(:,:,i) = Q(:,:,i) + A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*A(:,:,i);
        s(:,:,i) = A(:,:,i).'*(eye(size(Q,1)) - P(:,:,i+1)*M(:,:,i)*B(:,:,i)/R*B(:,:,i).')*s(:,:,i+1)+...
            A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*B(:,:,i)*uh0(:,i) - Q(:,:,i)*xh0(:,i);
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
    if norm(uh)<0.00001*norm(u)
        disp(['SLQ found the optimal control input within ',num2str(iter-1),' iterations'])
        for i = 2:size(t,2)
                q(:,i) = q(:,i-1) + x(4:6,i-1)*dt;
                Jac(:,:,i-1) = jacobiano3(q(:,i-1));
                x(1:3,i) = x(1:3,i-1) + Jac(:,:,i-1)*x(4:6,i-1)*dt;       
                x(4:6,i) = x(4:6,i-1) + u(:,i-1)*dt; 
        end
        break;
    else
        % Line search to optimize alfa
        alfa = 1:-lineSearchStep:0.0001;
        J = zeros(size(alfa));
        uk = u;
        for n = 1:size(alfa,2)
            u = uk + alfa(n)*uh;
            for i = 2:size(t,2)
                q(:,i) = q(:,i-1) + x(4:6,i-1)*dt;
                Jac(:,:,i-1) = jacobiano3(q(:,i-1));
                x(1:3,i) = x(1:3,i-1) + Jac(:,:,i-1)*x(4:6,i-1)*dt;       
                x(4:6,i) = x(4:6,i-1) + u(:,i-1)*dt; 
            end
            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Qend*(x(:,end)-x0(:,end));
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
    hold off;
    [T01, T02, T03] = directo3(q(:,1));
    plot([0 T01(1,4) T02(1,4) T03(1,4)],[0 T01(2,4) T02(2,4) T03(2,4)]);
    hold on;
    [T01, T02, T03] = directo3(q(:,end));
    plot([0 T01(1,4) T02(1,4) T03(1,4)],[0 T01(2,4) T02(2,4) T03(2,4)]);
    plot(x(1,:),x(2,:))
    title('State of the manipulator', 'interpreter', ...
    'latex','fontsize',18)
    xlabel('$x(m)$', 'interpreter', 'latex','fontsize',18)
    ylabel('$y(m)$', 'interpreter', 'latex','fontsize',18)

end


%% Results
toc

iu = cumsum(abs(u(1,:)));
disp(['Total acc applied joint 1: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(2,:)));
disp(['Total acc applied joint 2: ',num2str(iu(end)),' m/s^2'])
iu = cumsum(abs(u(3,:)));
disp(['Total acc applied joint 3: ',num2str(iu(end)),' m/s^2'])

figure(1)
hold off;
[T01, T02, T03] = directo3(q(:,1));
plot([0 T01(1,4) T02(1,4) T03(1,4)],[0 T01(2,4) T02(2,4) T03(2,4)]);
hold on;
[T01, T02, T03] = directo3(q(:,end));
plot([0 T01(1,4) T02(1,4) T03(1,4)],[0 T01(2,4) T02(2,4) T03(2,4)]);
plot(x(1,:),x(2,:))
title('State of the manipulator', 'interpreter', ...
'latex','fontsize',18)
xlabel('$x(m)$', 'interpreter', 'latex','fontsize',18)
ylabel('$y(m)$', 'interpreter', 'latex','fontsize',18)



figure(2)
plot(t,x)
title('Evolution of the state', 'interpreter', ...
'latex','fontsize',18)
legend('$x_{ee}$','$y_{ee}$','$\theta_{ee}$','$\tau_1^.$','$\tau_2^.$','$\tau_3^.$', 'interpreter', ...
'latex','fontsize',18)
xlabel('$t (s)$', 'interpreter', 'latex','fontsize',18)
ylabel('$\theta (rad)$', 'interpreter', 'latex','fontsize',18)
grid

figure(3)
plot(t,u)
title('Actuating acc (u)','interpreter','latex','fontsize',18)
xlabel('t(s)','interpreter','latex','fontsize',18)
ylabel('$a(m/s^2$)','interpreter','latex','fontsize',18)
legend('$Joint 1$','$Joint 2$','$Joint 3$', 'interpreter', ...
'latex','fontsize',18)
hold off



