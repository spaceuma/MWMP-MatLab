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
yf = 0.05;
vf = 0;

yMax = 3;
yMin = -3;

vMax = 2;
vMin = -2;

FMax = 1;
FMin = -1;

dt = 0.05;
t = 0:dt:tf;

% State vector
numStates = 2;
x = zeros(numStates,size(t,2));
x(1,1) = yi;
x(2,1) = vi;

xs = zeros(numStates,size(t,2));

% Initial control law
numInputs = 1;
u = zeros(numInputs,size(t,2));

us = zeros(numInputs,size(t,2));

% Target state and control trajectories
x0 = zeros(numStates,size(t,2));
x0(1,end) = yf;
x0(2,end) = vf;

u0 = zeros(numInputs,size(t,2));

% Constraints matrices definition
% State input constraints
numStateInputConstraints = 2;
I = zeros(numStateInputConstraints,size(t,2));
C = zeros(numStateInputConstraints,numStates,size(t,2));
D = zeros(numStateInputConstraints,numInputs,size(t,2));
r = zeros(numStateInputConstraints,size(t,2));

D(1,1,:) = 1;
r(1,:) = -FMax;

D(2,1,:) = -1;
r(2,:) = FMin;

% Pure state constraints
numPureStateConstraints = 4;
J = zeros(numPureStateConstraints,size(t,2));
G = zeros(numPureStateConstraints,numStates,size(t,2));
h = zeros(numPureStateConstraints,size(t,2));

G(1,1,:) = 1;
h(1,:) = -yMax;

G(2,1,:) = -1;
h(2,:) = yMin;

G(3,2,:) = 1;
h(3,:) = -vMax;

G(4,2,:) = -1;
h(4,:) = vMin;


% PRUEBA
I(1,[48 53 68]) = 1;

J(1,[145 150]) = 1;

J(4,200) = 1;

% SLQR algorithm
iter = 1;
while 1   
%     % Forward integrate system equations
%     for i = 2:size(t,2)
%         a = (uk(i-1) - springK*xk(1,i-1) - damperB*xk(2,i-1))/mass;
%         xk(1,i) = xk(1,i-1) + dt*xk(2,i-1);
%         xk(2,i) = xk(2,i-1) + dt*a;
%     end
        
    % Quadratize cost function along the trajectory
    Q = zeros(numStates,numStates,size(t,2));
    for i = 1:size(t,2)
        Q(:,:,i) = 0*eye(numStates,numStates);
    end
    
    Q(:,:,end) = [10000000 0;
                  0 10000000];
        
    R = zeros(numInputs,numInputs,size(t,2));
    for i = 1:size(t,2)
        R(:,:,i) = 1*eye(numInputs,numInputs);
    end
    
    % Define the sequential state and input vectors
    xs0 = zeros(numStates,size(t,2));
    us0 = zeros(numInputs,size(t,2));

    for i = 1:size(t,2)
        xs0(:,i) = Q(:,:,i)*(x(:,i) - x0(:,i));
        us0(:,i) = R(:,:,i)*(u(:,i) - u0(:,i));
        
        % Según Sideris, debería ser como sigue, aunque con esto no estamos
        % realimentando el vector de referencia
%         xs0(:,i) = Q(:,:,i)*x(:,i) + x0(:,i);
%         us0(:,i) = R(:,:,i)*u(:,i) + u0(:,i);
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
    
    % Active Constraints definition
    % State input constraints
    tl = [];
    p = [];
    paux = zeros(numStateInputConstraints,1);

    for i = 1:size(t,2)
        anyActiveConstraint = false;
        for j = 1:numStateInputConstraints
            if I(j,i)
                paux(j) = 1;
                anyActiveConstraint = true;
            end
        end
        if anyActiveConstraint
            tl = [tl i];
        end
    end
    for i = 1:numStateInputConstraints
        if paux(i)
            p = [p i];
        end
    end
    numActiveSIConstraints = size(p,2);

    Cl = zeros(numActiveSIConstraints,numStates,size(t,2));
    Dl = zeros(numActiveSIConstraints,numInputs,size(t,2));
    rl = zeros(numActiveSIConstraints,size(t,2));
    
    if(numActiveSIConstraints)
        Cl(:,:,tl) = C(p,:,tl);
        Dl(:,:,tl) = D(p,:,tl);
        rl(:,tl) = r(p,tl);
    end
    
    
    % Pure state constraints
    tk = [];
    q = [];    
    qaux = zeros(numPureStateConstraints,1);


    for i = 1:size(t,2)
        anyActiveConstraint = false;
        for j = 1:numPureStateConstraints
            if J(j,i)
                qaux(j) = 1;
                anyActiveConstraint = true;
            end
        end
        if anyActiveConstraint
            tk = [tk i];
        end
    end
    for i = 1:numPureStateConstraints
        if qaux(i)
            q = [q i];
        end
    end
    numActivePSConstraints = size(q,2);
    
    
    Gk = zeros(numActivePSConstraints,numStates,size(t,2));
    hk = zeros(numActivePSConstraints,size(t,2));
    
    if(numActivePSConstraints)
        Gk(:,:,tk) = G(q,:,tk);
        hk(:,tk) = h(q,tk);
    end
    
    % Predefinitions
    Dh = zeros(numActiveSIConstraints,numActiveSIConstraints,size(t,2));
    E = zeros(numActiveSIConstraints,numStates,size(t,2));
    rh = zeros(numActiveSIConstraints,size(t,2));
    
    Ah = zeros(numStates,numStates,size(t,2));
    Rh = zeros(numStates,numStates,size(t,2));
    Qh = zeros(numStates,numStates,size(t,2));
    x0h = zeros(numStates,size(t,2));
    u0h = zeros(numStates,size(t,2));
    
    for i = 1:size(tl,2)
        Dh(:,:,tl(i)) = inv(Dl(:,:,tl(i))/R(:,:,tl(i))*Dl(:,:,tl(i)).');
    end
    
    for i = 1:size(t,2)

        E(:,:,i) = Cl(:,:,i);
        rh(:,i) = rl(:,i) - Dl(:,:,i)/R(:,:,i)*us0(:,i);
        
        Ah(:,:,i) = A(:,:,i) - B(:,:,i)/R(:,:,i)*Dl(:,:,i).'*Dh(:,:,i)*E(:,:,i);
        Rh(:,:,i) = B(:,:,i)/R(:,:,i)*(eye(numInputs,numInputs)-Dl(:,:,i).'*Dh(:,:,i)*Dl(:,:,i)/R(:,:,i))*B(:,:,i).';
        Qh(:,:,i) = Q(:,:,i) + E(:,:,i).'*Dh(:,:,i)*E(:,:,i);
        x0h(:,i) = xs0(:,i) + E(:,:,i).'*Dh(:,:,i)*rh(:,i);
        u0h(:,i) = -B(:,:,i)/R(:,:,i)*(us0(:,i) + Dl(:,:,i).'*Dh(:,:,i)*rh(:,i));
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
    
    Gamma = zeros(numActivePSConstraints*size(tk,2),numStates);
    Gammak = zeros(numActivePSConstraints,numStates,size(t,2), size(tk,2));
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

        for j = 1:size(tk,2)
            minConstraintIndex = min(tk(k),tk(j));
            Fkj = zeros(numActivePSConstraints,numActivePSConstraints,minConstraintIndex);
            for n = minConstraintIndex-1:-1:1
                Fkj(:,:,n) = Fkj(:,:,n+1) - Gammak(:,:,n+1,k)*M(:,:,n)*Rh(:,:,n)*Gammak(:,:,n+1,j).';
            end
            F((k-1)*numActivePSConstraints+1:k*numActivePSConstraints,...
              (j-1)*numActivePSConstraints+1:j*numActivePSConstraints) = Fkj(:,:,1);
        end

        H((k-1)*numActivePSConstraints+1:k*numActivePSConstraints) = hk(:,tk(k));
        Gamma((k-1)*numActivePSConstraints+1:k*numActivePSConstraints,:) = Gammak(:,:,1,k);
        y((k-1)*numActivePSConstraints+1:k*numActivePSConstraints) = yk(:,1);        

    end
    
    nu = zeros(numActivePSConstraints*size(tk,2),1);
    invF = zeros(size(F));
    if(~det(F))
        Faux = F;
        currentIndex = 0;
        correctIndex = [];
        for i = tk
            for j = 1:numActivePSConstraints
                if(J(q(j),i))
                    correctIndex = [correctIndex currentIndex+j];
                end
            end
            currentIndex = currentIndex + numActivePSConstraints;
        end
        
        Faux = F(correctIndex,correctIndex);
        invFaux = inv(Faux);
        invF(correctIndex,correctIndex) = invFaux;
    else
        invF = inv(F);
    end
    nu(:) = invF*(-Gamma*xs(:,1) - y - H);

    s = zeros(numStates,size(t,2));

    for i = 1:size(t,2)
        sum = 0;
        for k = 1:size(tk,2)
            if (tk(k) > i)
                sum = sum + Gammak(:,:,i,k).'*nu((k-1)*numActivePSConstraints+1:k*numActivePSConstraints);
            end
        end
        s(:,i) = z(:,i) + sum;
    end
    
    v = zeros(numStates,size(t,2));
    lambda = zeros(numStates,size(t,2));
    mu = zeros(numActiveSIConstraints,size(t,2));
    
%     xsk(:,1) = zeros(numStates,1);
%     xs(:,1) = xs0(:,1);

    % Solve forward
    for i = 1:size(t,2)-1
        v(:,i) = M(:,:,i)*(u0h(:,i) - Rh(:,:,i)*s(:,i+1));
        xs(:,i+1) = M(:,:,i)*Ah(:,:,i)*xs(:,i) + v(:,i);
        lambda(:,i+1) = P(:,:,i+1)*xs(:,i+1) + s(:,i+1);
        mu(:,i) = Dh(:,:,i)*(E(:,:,i)*xs(:,i) - Dl(:,:,i)/R(:,:,i)*B(:,:,i).'*lambda(:,i+1) + rh(:,i));
        us(:,i) = -R(:,:,i)\(B(:,:,i).'*lambda(:,i+1) + Dl(:,:,i).'*mu(:,i) + us0(:,i));
    end
   
    % Exit condition
    step3 = true;
    if norm(us)>0.000001*norm(u)
        % Step 2

        
        % Update controller
%         x = x + alfak*xs;
%         u = u + alfak*us;

%         
%         if alfak == 1
%             step3 = true;
%         else
%             step3 = false;
%         end
    end
    if step3
        % Step 3
        
        if iter >= 2
            iter = iter-1;
            break;
        end
    end   
    x = x + xs;
    u = u + us;
    iter = iter+1;
end
disp(['SLQ found the optimal control input within ',num2str(iter),' iterations'])
toc

iu = cumsum(abs(u));
disp(['Total force applied: ',num2str(iu(end)),' N'])

figure(1)
plot(t,x(1,:))
title('Mass position evolution','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('y(m)','interpreter','latex')
hold on
plot(tf,yf,'Marker','o','MarkerFaceColor','red')
% plot((tk(1)-1)*dt,-hk(1,tk(1)),'Marker','o','MarkerFaceColor','red')
hold off

figure(2)
plot(t,x(2,:))
title('Mass speed evolution','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('v(m/s)','interpreter','latex')
hold on
plot(tf,vf,'Marker','o','MarkerFaceColor','red')
% plot((tk(2)-1)*dt,-hk(1,tk(2)),'Marker','o','MarkerFaceColor','red')
hold off

figure(3)
plot(t,u)
title('Actuating force (u)','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('F(N)','interpreter','latex')
hold on
% plot((tl(1)-1)*dt,-rl(1,tl(1)),'Marker','o','MarkerFaceColor','red')
hold off


