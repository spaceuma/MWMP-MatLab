tic

% Constraints
ti = 0;
tf = 1;

dt = 0.01;
t = 0:dt:tf;

% State vector
x = zeros(2,size(t,2));
x(1,1) = 0;
x(2,1) = -1;

% Initial control law
u = zeros(1,size(t,2));

% Target state and control trajectories
x0 = zeros(2,size(t,2));
u0 = zeros(1,size(t,2));

% SLQR algorithm
lineSearchStep = 0.2;
iter = 1;
while 1   
    % Forward integrate system equations
    for i = 2:size(t,2)
        x(1,i) = x(1,i-1) + dt*x(2,i-1);
        x(2,i) = x(2,i-1)*(1-dt) + dt*u(1,i);
    end
    
    xh0 = x0 - x;
    uh0 = u0 - u;
    
    % Quadratize cost function along the trajectory
    Q = zeros(size(x,1),size(x,1),size(t,2));
    for i = 1:size(t,2)
        Q(:,:,i) = [0.0200 0.0001;
                    0.0001 0.0198];
    end

    R = 1.0066e-4;

    % Linearize the system dynamics and constraints along the trajectory  
    A = zeros(size(x,1),size(x,1),size(t,2));
    for i = 1:size(t,2)
        A(:,:,i) = [1 0.01
                    0 0.99]; 
    end
     
    B = zeros(size(x,1),size(u,1),size(t,2));
    for i = 1:size(t,2)
        B(:,:,i) = [0.00;
                    0.01]; 
    end
    
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
    P(:,:,end) = Q(:,:,end);
    s(:,:,end) = -Q(:,:,end)*xh0(:,end);
    
    xh = zeros(size(x,1),size(t,2));
    uh = zeros(size(u,1),size(t,2));
    v = zeros(size(xh));
    lambdah = zeros(size(s));    
%     xh(1,1) = yi;
%     xh(2,1) = vi;

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
    
    
    % Exit condition
    if norm(uh)<0.0001*norm(u)
        break;
    else
        % Line search to optimize alfa
        alfa = 1:-lineSearchStep:0.0001;
        J = zeros(size(alfa));
        uk = u;
        for n = 1:size(alfa,2)
            u = uk + alfa(n)*uh;
            for i = 2:size(t,2)
                x(1,i) = x(1,i-1) + dt*x(2,i-1);
                x(2,i) = x(2,i-1)*(1-dt) + dt*u(1,i);
            end
            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Q(:,:,end)*(x(:,end)-x0(:,end));
            for i = 1:size(t,2)-1
                J(n) = J(n) + 1/2*((x(:,i)-x0(:,i)).'*Q(:,:,i)*(x(:,i)-x0(:,i))...
                    + (u(:,i)-u0(:,i)).'*R*(u(:,i)-u0(:,i)));
            end            
        end
        [mincost, ind] = min(J);
        alfamin = alfa(ind);
        
        % Update controller
        u = uk + alfamin*uh;
        iter = iter+1;
    end   

end
disp(['SLQ found the optimal control input within ',num2str(iter),' iterations'])
toc

for i = 2:size(t,2)
    x(1,i) = x(1,i-1) + dt*x(2,i-1);
    x(2,i) = x(2,i-1)*(1-dt) + dt*u(1,i);
end

h = zeros(1,size(t,2));
for k = 1:size(t,2)
    h(1,k) = -8*((k+1)*dt-0.5)^2+0.5;
end

figure
subplot(2,1,1)
plot(t,x(1,:),'Color','r')
hold on
plot(t,x(2,:),'b')
plot(t,-h,'g')
legend('$x_1$','$x_2$','State constraint','interpreter','latex')
title('State constrained','interpreter','latex')
xlabel('Time','interpreter','latex')
ylabel('Magnitude','interpreter','latex')
axis([0 1 -1 1])
hold off
subplot(2,1,2)
plot(t,u)
title('Control trajectory','interpreter','latex')
xlabel('Time','interpreter','latex')
ylabel('Magnitude','interpreter','latex')
hold off



