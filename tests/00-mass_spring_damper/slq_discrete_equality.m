init = cputime;
% System properties
m = 1;
k = 1;
b = 0.2;

ti = 0;
yi = 0;
vi = 0;

tf = 15;
dt = 0.01;
t = 0:dt:tf;

% State vector
x = zeros(2,size(t,2));
x(1,1) = yi;
x(2,1) = vi;

% Initial control law
u = zeros(1,size(t,2));

% Target state and control trajectories
x0 = zeros(2,size(t,2)).';
x0(end,1) = 1;
x0(end,2) = 0;

u0 = zeros(1,size(t,2)).';

% Constraints
% State-input constraints
Cxu = [0; 15];
C = zeros(size(x,1),size(Cxu));
D = zeros(size(u,1),size(Cxu));
r = zeros(1,size(Cxu));

% Pure state constraints
Cx = [5 10 15];

G = zeros(size(x,1),size(Cx));
 
h = [2; 0.5; 1;
     0; 0.5; 0];

% SLQR algorithm
iter = 1;
while 1   
    % Forward integrate system equations
    for i = 2:size(t,2)
        a = (u(i-1) - k*x(1,i-1) - b*x(2,i-1))/m;
        x(2,i) = x(2,i-1) + dt*a;
        x(1,i) = x(1,i-1) + dt*x(2,i);
    end
        
    % Quadratize cost function along the trajectory
    Q = [1 0;
         0 1];
    Qend = [1000000 0;
            0  1000000];
    R = 1;
    K = 0;

    % Linearize the system dynamics and constraints along the trajectory    
    A = [1        dt;
         -dt*k/m  1-dt*b/m]; 
    B = [0;
         dt/m]; 
    
    % LQ problem solution
    M = zeros(size(B,1),size(B,1),size(t,2));
    P = zeros(size(Q,1),size(Q,2),size(t,2));
    z = zeros(size(Q,1),1,size(t,2));
    P(:,:,end) = Qend;
    z(:,:,end) = x0(end,:);
    
    xh = zeros(2,size(t,2));
    uh = zeros(1,size(t,2));
    v = zeros(size(xh));
    lambdah = zeros(size(z));    
    xh(1,1) = yi;
    xh(2,1) = vi;
    
    % Solve backward
    for i = size(t,2)-1:-1:1
        % Definitions
        Dh = (D(i)*R^-1*D(i).')^-1;
        if Dh == Inf 
            Dh = 0;
        end
        E = C - D(i)*R^-1*K.';
        rh = r(i) - D(i)*R^-1*u0(i);
    
        Ah = A - B*R^-1*(K.'+D(i).'*Dh*E);
        Rh = B*R^-1*(eye(1)-D(i).'*Dh*D(i)*R^-1)*B.';
        Qh = Q - K*R^-1*K.' + E.'*Dh*E;
    
        xh0 = x0(i) - K*R^-1*u0(i) + E.'*Dh*rh;
        uh0 = -B*R^-1*(u0(i) + D(i).'*Dh*rh);       
        
        M(:,:,i) = inv(eye(size(B,1)) + Rh*P(:,:,i+1));
        P(:,:,i) = Qh + Ah.'*P(:,:,i+1)*M(:,:,i)*Ah;
        z(:,:,i) = Ah.'*M(:,:,i).'*z(:,:,i+1)+Ah.'*P(:,:,i+1)*M(:,:,i)*uh0+xh0;
    end
    
    % Solve forward
    for i = 1:size(t,2)-1
        v(:,i) = M(:,:,i)*B*(uh0(i)-R*B.'*s(:,:,i+1));
        xh(:,i+1) = M(:,:,i)*A*xh(:,i)+v(:,i);
        lambdah(:,:,i+1) = P(:,:,i+1)*xh(:,i+1)+s(:,:,i+1);
        uh(i) = uh0(i)-inv(R)*B.'*lambdah(:,:,i+1);
    end
    
    % Exit condition
    if norm(uh)<0.01*norm(u)
        break;
    else
        % Line search to optimize alfa
        alfa = 1:-0.01:0.0001;
        J = zeros(size(alfa));
        uk = u;
        for n = 1:size(alfa,2)
            u = uk + alfa(n)*uh;
            for i = 2:size(t,2)
                a = (u(i-1) - k*x(1,i-1) - b*x(2,i-1))/m;
                x(2,i) = x(2,i-1) + dt*a;
                x(1,i) = x(1,i-1) + dt*x(2,i);
            end
            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Qend*(x(:,end)-x0(:,end));
            for i = 1:size(t,2)-1
                J(n) = J(n) + 1/2*((x(:,i)-x0(:,i)).'*Q*(x(:,i)-x0(:,i)) + (u(:,i)-u0(:,i)).'*R*(u(:,i)-u0(:,i)));
            end            
        end
        [mincost, ind] = min(J);
        alfamin = alfa(ind);
        
        % Update controller
        u = uk + alfamin*uh;
    end   

    iter = iter+1;
end
endt = cputime;
disp(['SLQ found the optimal control input within ',num2str(iter),' iterations'])
disp(['Elapsed execution time: ',num2str(endt-init),' seconds'])
iu = cumsum(u);
disp(['Total force applied: ',num2str(iu(end)),' N*s'])

figure(1)
plot(t,x(1,:))
title('Mass position evolution','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('y(m)','interpreter','latex')
hold on
plot(tf,yf,'Marker','o','MarkerFaceColor','red')

figure(2)
plot(t,x(2,:))
title('Mass speed evolution','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('v(m/s)','interpreter','latex')
hold on
plot(tf,vf,'Marker','o','MarkerFaceColor','red')

figure(3)
plot(t,u)
title('Actuating force (u)','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('F(N)','interpreter','latex')
hold on



