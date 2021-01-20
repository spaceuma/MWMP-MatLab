tic
lineSearchStep = 0.2;

% System properties
m = 1;
k = 10;
b = 2;

% Constraints
ti = 0;
yi = 0;
vi = 0;

tf = 15;
yf = 0;
vf = -18;

g = 9.81;
 
dt = 0.05;
t = 0:dt:tf;

% State vector
x = zeros(2,size(t,2));
x(1,1) = yi;
x(2,1) = vi;

% Initial control law
u = zeros(1,size(t,2)-1);

% Target state and control trajectories
x0 = zeros(2,size(t,2));
x0(1,end) = yf;
x0(2,end) = vf;

u0 = zeros(1,size(t,2)-1);

% SLQR algorithm
iter = 1;
while 1   
    % Forward integrate system equations
    for i = 2:size(t,2)
        a = (u(i-1) - k*x(1,i-1) - b*x(2,i-1))/m;
        x(1,i) = x(1,i-1) + dt*x(2,i-1);
        x(2,i) = x(2,i-1) + dt*a;
    end
    
    xh0 = x0 - x;
    uh0 = u0 - u;
    
    % Quadratize cost function along the trajectory
    Q = zeros(size(x,1),size(x,1),size(t,2));
    for i = 1:size(t,2)
        Q(:,:,i) = zeros(size(x,1));
    end
    
    Qend = [10000000000000 0;
            0  10000000000000];
    R = 1;

    % Linearize the system dynamics and constraints along the trajectory  
    A = zeros(size(x,1),size(x,1),size(t,2));
    for i = 1:size(t,2)
        A(:,:,i) = [1        dt;
                    -dt*k/m  1-dt*b/m]; 
    end
     
    B = zeros(size(x,1),size(u,1),size(t,2));
    for i = 1:size(t,2)
        B(:,:,i) = [0;
                    dt/m]; 
    end
    
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    s = zeros(size(Q,1),1,size(t,2));
    P(:,:,end) = Qend;
    s(:,:,end) = -Qend*xh0(:,end);
    
    xh = zeros(size(x,1),size(t,2));
    uh = zeros(size(u,1),size(t,2)-1);
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
                a = (u(i-1) - k*x(1,i-1) - b*x(2,i-1))/m;
                x(1,i) = x(1,i-1) + dt*x(2,i-1);
                x(2,i) = x(2,i-1) + dt*a;
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
        iter = iter+1;
    end   

end
disp(['SLQ found the optimal control input within ',num2str(iter),' iterations'])
toc
iu = cumsum(abs(u));
disp(['Total force applied: ',num2str(iu(end)),' N'])

figure(4)
plot(t,x(1,:))
title('Mass position evolution','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('y(m)','interpreter','latex')
hold on
plot(tf,yf,'Marker','o','MarkerFaceColor','red')
hold off

figure(5)
plot(t,x(2,:))
title('Mass speed evolution','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('v(m/s)','interpreter','latex')
hold on
plot(tf,vf,'Marker','o','MarkerFaceColor','red')
hold off

figure(6)
plot(t(1:end-1),u)
title('Actuating force (u)','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('F(N)','interpreter','latex')
hold off



