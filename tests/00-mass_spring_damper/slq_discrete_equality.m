clear

tic
lineSearchStep = 0.2;

% System properties
mass = 1;
springK = 10;
damperB = 2;

% Constraints
ti = 0;
yi = 0;
vi = 0;

tf = 15;
yf = 1;
vf = 0;

g = 9.81;
 
dt = 0.05;
t = 0:dt:tf;

% State vector
numStates = 2;
x = zeros(numStates,size(t,2));
x(1,1) = yi;
x(2,1) = vi;

xk = x;
xsk = zeros(numStates,size(t,2));

% Initial control law
numInputs = 1;
u = zeros(numInputs,size(t,2));

uk = u;
usk = zeros(numInputs,size(t,2));

% Target state and control trajectories
x0 = zeros(numStates,size(t,2));
x0(1,end) = yf;
x0(2,end) = vf;

u0 = zeros(numInputs,size(t,2));

% SLQR algorithm
iter = 1;
while 1   
    % Forward integrate system equations
    for i = 2:size(t,2)
        a = (uk(i-1) - springK*xk(1,i-1) - damperB*xk(2,i-1))/mass;
        xk(1,i) = xk(1,i-1) + dt*xk(2,i-1);
        xk(2,i) = xk(2,i-1) + dt*a;
    end
        
    % Quadratize cost function along the trajectory
    Q = zeros(numStates,numStates,size(t,2));
    for i = 1:size(t,2)
        Q(:,:,i) = 0*eye(numStates,numStates);
    end
    
    Q(:,:,end) = [100000 0;
                  0  100000];
        
    R = eye(numInputs);
    
    % Define the sequential state and input vectors
    xs0 = zeros(numStates,size(t,2));
    us0 = zeros(numInputs,size(t,2));

    for i = 1:size(t,2)
        xs0(:,i) = Q(:,:,i)*xk(:,i) + x0(:,i);
        us0(:,i) = uk(:,i) + u0(:,i);

%         xs0(:,i) = -xk(:,i) + x0(:,i);
%         us0(:,i) = -uk(:,i) + u0(:,i);
    end
    

    % Linearize the system dynamics and constraints along the trajectory  
    A = zeros(numStates,numStates,size(t,2));
    for i = 1:size(t,2)
        A(:,:,i) = [1        dt;
                    -dt*springK/mass  1-dt*damperB/mass]; 
    end
     
    B = zeros(numStates,numInputs,size(t,2));
    for i = 1:size(t,2)
        B(:,:,i) = [0;
                    dt/mass]; 
    end
    
    % Constraints definition
    % State input constraints
    numStateInputConstraints = 0;
    tsSI = zeros(numStateInputConstraints,1);
    C = zeros(numStateInputConstraints,numStates,size(t,2));
    D = zeros(numStateInputConstraints,numInputs,size(t,2));
    r = zeros(numStateInputConstraints,size(t,2));
    
%     tsSI(1) = 100;
%     D(1,1,tsSI(1)) = 1;
%     r(1,tsSI(1)) = -1;
%     
%     tsSI(2) = 200;
%     D(2,1,tsSI(2)) = 1;
%     r(2,tsSI(2)) = -1;
%     
%     tsSI(3) = 300;
%     D(1,1,tsSI(3)) = 1;
%     r(1,tsSI(3)) = -1;
    
    % Pure state constraints
    numPureStateConstraints = 0;
    tsPS = zeros(numPureStateConstraints,1);
    G = zeros(numPureStateConstraints,numStates,size(t,2));
    h = zeros(numPureStateConstraints,size(t,2));
    
    % Speed has to be 1 m/s at timestep 500
%     tsPS(1) = 500;
%     G(1,2,tsPS(1)) = 1;
%     h(1,tsPS(1)) = -vf;

%     tsPS(2) = size(t,2);
%     G(2,2,tsPS(2)) = 1;
%     h(2,tsPS(2)) = -vf;
    
    % Predefinitions
    Dh = zeros(numStateInputConstraints,numStateInputConstraints,size(t,2));
    E = zeros(numStateInputConstraints,numStates,size(t,2));
    rh = zeros(numStateInputConstraints,size(t,2));
    
    Ah = zeros(numStates,numStates,size(t,2));
    Rh = zeros(numStates,numStates,size(t,2));
    Qh = zeros(numStates,numStates,size(t,2));
    x0h = zeros(numStates,size(t,2));
    u0h = zeros(numStates,size(t,2));
    
    for k = 1:numStateInputConstraints
        Dh(:,:,tsSI(k)) = inv(D(:,:,tsSI(k))/R*D(:,:,tsSI(k)).');
    end
    
    
    for i = 1:size(t,2)
        E(:,:,i) = C(:,:,i);
        rh(:,i) = r(:,i) - D(:,:,i)/R*us0(:,i);
        
        Ah(:,:,i) = A(:,:,i) - B(:,:,i)/R*D(:,:,i).'*Dh(:,:,i)*E(:,:,i);
        Rh(:,:,i) = B(:,:,i)/R*(eye(numInputs,numInputs)-D(:,:,i).'*Dh(:,:,i)*D(:,:,i)/R)*B(:,:,i).';
        Qh(:,:,i) = Q(:,:,i) + E(:,:,i).'*Dh(:,:,i)*E(:,:,i);
        x0h(:,i) = xs0(:,i) + E(:,:,i).'*Dh(:,:,i)*rh(:,i);
        u0h(:,i) = -B(:,:,i)/R*(us0(:,i) + D(:,:,i).'*Dh(:,:,i)*rh(:,i));
    end
    
    % LQ problem solution
    M = zeros(numStates,numStates,size(t,2));
    P = zeros(numStates,numStates,size(t,2));
    z = zeros(numStates,size(t,2));
    P(:,:,end) = Q(:,:,end);
    z(:,end) = xs0(:,end);
    
    % Solve backward
    for i = size(t,2)-1:-1:1
        M(:,:,i) = inv(eye(numStates) + Rh(:,:,i)*P(:,:,i+1));
        P(:,:,i) = Qh(:,:,i) + Ah(:,:,i).'*P(:,:,i+1)*M(:,:,i)*Ah(:,:,i);
        z(:,i) = Ah(:,:,i).'*M(:,:,i).'*z(:,i+1) + Ah(:,:,i).'*P(:,:,i+1)*M(:,:,i)*u0h(:,i) + x0h(:,i);
    end
    
    Gamma = zeros(numPureStateConstraints,numStates,size(t,2));
    y = zeros(numPureStateConstraints,1);
    F = zeros(numPureStateConstraints,numPureStateConstraints);
    H = zeros(numPureStateConstraints,1);

    % Solve over all state constraints   
    for k = 1:numPureStateConstraints
        yk = zeros(1,tsPS(k));
        
        Gamma(k,:,tsPS(k)) = G(k,:,tsPS(k));
        yk(tsPS(k)) = 0;
        
        for n = tsPS(k)-1:-1:1
            Gamma(k,:,n) = Gamma(k,:,n+1)*M(:,:,n)*Ah(:,:,n);
            yk(n) = yk(n+1) + Gamma(k,:,n+1)*M(:,:,n)*(u0h(:,n) - Rh(:,:,n) * z(:,n+1));
        end
        y(k) = yk(1);        
        
        for j = 1:numPureStateConstraints           
            minConstraintIndex = min(tsPS(k),tsPS(j));
            Fkj = zeros(1,minConstraintIndex);
            for n = minConstraintIndex-1:-1:1
                Fkj(n) = Fkj(n+1) - Gamma(k,:,n+1)*M(:,:,n)*Rh(:,:,n)*Gamma(j,:,n+1).';
            end
            F(k,j) = Fkj(1);
        end
        
        H(k) = h(k,tsPS(k));
    end
    
    nu = zeros(numPureStateConstraints,1);
    nu(:) = F\(-Gamma(:,:,1)*xs0(:,1) - y - H);
    
    s = zeros(numStates,size(t,2));

    for i = 1:size(t,2)
        sum = 0;
        for k = 1:numPureStateConstraints
            if (tsPS(k) > i)
                sum = sum + Gamma(k,:,n).'*nu(k);
            end
        end
        s(:,i) = z(:,i) + sum;
    end
    
    v = zeros(numStates,size(t,2));
    lambda = zeros(numStates,size(t,2));
    mu = zeros(numStateInputConstraints,size(t,2));
    
    xsk(:,1) = xs0(:,1);
    
    % Solve forward
    for i = 1:size(t,2)-1
        v(:,i) = M(:,:,i)*(u0h(:,i) - Rh(:,:,i)*s(:,i));
        xsk(:,i+1) = M(:,:,i)*Ah(:,:,i)*xsk(:,i) + v(:,i);
        lambda(:,i+1) = P(:,:,i+1)*xsk(:,i+1) + s(:,i+1);
        mu(:,i) = Dh(:,:,i)*(E(:,:,i)*xsk(:,i) - D(:,:,i)/R*B(:,:,i).'*lambda(:,i+1) + rh(:,i));
        usk(:,i) = -R\(B(:,:,i).'*lambda(:,i+1) + D(:,:,i).'*mu(:,i) + us0(:,i));
    end
   
    % Exit condition
    if norm(usk)<0.001*norm(uk)
%     if iter > 50
        break;
    else
        % Line search to optimize alfa
        alfa = 1:-lineSearchStep:0.0001;
        J = zeros(size(alfa));
        for n = 1:size(alfa,2)
            xkaux = xk + alfa(n)*xsk;
            ukaux = uk + alfa(n)*usk;
            J(n) = 1/2*(xkaux(:,end)-x0(:,end)).'*Q(:,:,end)*(xkaux(:,end)-x0(:,end));
            for i = 1:size(t,2)-1
                J(n) = J(n) + 1/2*((xkaux(:,i)-x0(:,i)).'*Q(:,:,i)*(xkaux(:,i)-x0(:,i))...
                    + (ukaux(:,i)-u0(:,i)).'*R*(ukaux(:,i)-u0(:,i)));
            end            
        end
        [mincost, ind] = min(J);
        alfamin = alfa(ind);
        
        % Update controller
        xk = xk + alfamin*xsk;
        uk = uk + alfamin*usk;
%         xk = xsk;
%         uk = usk;

        iter = iter+1;
    end   
end
disp(['SLQ found the optimal control input within ',num2str(iter),' iterations'])
toc
x = xk;
u = uk;

iu = cumsum(abs(u));
disp(['Total force applied: ',num2str(iu(end)),' N'])

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



