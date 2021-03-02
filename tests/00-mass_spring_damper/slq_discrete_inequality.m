clear
tic

% System properties
mass = 1;
springK = 10;
damperB = 2;

% Constraints
ti = 0;
yi = 0;
vi = 0;

tf = 15;
yf = 0.15;
vf = 0.1;

yMax = 1;
yMin = -1;

vMax = 0.15;
vMin = -0.15;

FMax = 10;
FMin = -10;

dt = 0.05;
t = 0:dt:tf;

% State vector
numStates = 2;
x = zeros(numStates,size(t,2));
x(1,1) = yi;
x(2,1) = vi;

% Initial control law
numInputs = 1;
u = zeros(numInputs,size(t,2));

% Forward integrate system dynamics
for i = 2:size(t,2)
    a = (u(i-1) - springK*x(1,i-1) - damperB*x(2,i-1))/mass;
    x(1,i) = x(1,i-1) + dt*x(2,i-1);
    x(2,i) = x(2,i-1) + dt*a;
end

% Target state and control trajectories
x0 = zeros(numStates,size(t,2));
x0(1,end) = yf;
x0(2,end) = vf;

u0 = zeros(numInputs,size(t,2));

% Constraints matrices definition
% State input constraints
numStateInputConstraints = 2;
I0 = zeros(numStateInputConstraints,size(t,2));
I = I0;
C = zeros(numStateInputConstraints,numStates,size(t,2));
D = zeros(numStateInputConstraints,numInputs,size(t,2));
r = zeros(numStateInputConstraints,size(t,2));

D(1,1,:) = 1;
r(1,:) = -FMax;

D(2,1,:) = -1;
r(2,:) = FMin;

% Pure state constraints
numPureStateConstraints = 4;
J0 = zeros(numPureStateConstraints,size(t,2));
J = J0;
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

% SLQR algorithm
iter = 0;
while 1   
    figure(1)
    plot(t,x(1,:))
    title('Mass position evolution','interpreter','latex')
    xlabel('t(s)','interpreter','latex')
    ylabel('y(m)','interpreter','latex')
    hold on
    plot(tf,yf,'Marker','o','MarkerFaceColor','red')
    hold off

    figure(2)
    plot(t,x(2,:))
    title('Mass speed evolution','interpreter','latex')
    xlabel('t(s)','interpreter','latex')
    ylabel('v(m/s)','interpreter','latex')
    hold on
    plot(tf,vf,'Marker','o','MarkerFaceColor','red')
    hold off

    figure(3)
    plot(t,u)
    title('Actuating force (u)','interpreter','latex')
    xlabel('t(s)','interpreter','latex')
    ylabel('F(N)','interpreter','latex')
    hold on
    hold off
    
    
    xs = zeros(numStates,size(t,2));
    us = zeros(numInputs,size(t,2));
        
    % Quadratize cost function along the trajectory
    Q = zeros(numStates,numStates,size(t,2));
    for i = 1:size(t,2)
        Q(:,:,i) = 0*eye(numStates,numStates);
    end
    
    Q(:,:,end) = [1000000000 0;
                  0 1000000000];
        
    R = zeros(numInputs,numInputs,size(t,2));
    for i = 1:size(t,2)
        R(:,:,i) = 1*eye(numInputs,numInputs);
    end
    
    K = zeros(numStates,numInputs,size(t,2));
    
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
    p = cell(size(t,2),1);
    paux = zeros(numStateInputConstraints,1);
    pl = zeros(numStateInputConstraints,1);
    
    for i = 1:size(t,2)
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
    
    Cl = zeros(numActiveSIConstraints,numStates,size(t,2));
    Dl = zeros(numActiveSIConstraints,numInputs,size(t,2));
    rl = zeros(numActiveSIConstraints,size(t,2));
    
    for i = tl
        Cl(pl(p{i}),:,i) = C(p{i},:,i);
        Dl(pl(p{i}),:,i) = D(p{i},:,i);
%         rl(pl(p{i}),i) = r(p{i},i);
    end
    
    
    % Pure state constraints
    tk = [];
    q = cell(size(t,2),1);
    qaux = zeros(numPureStateConstraints,1);
    ql = zeros(numPureStateConstraints,1);

    for i = 1:size(t,2)
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
    
    
    Gk = zeros(numActivePSConstraints,numStates,size(t,2));
    hk = zeros(numActivePSConstraints,size(t,2));
    
    for i = tk
        Gk(ql(q{i}),:,i) = G(q{i},:,i);
%         hk(ql(q{i}),i) = h(q{i},i);
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
    
    for i = tl
        Dh(pl(p{i}),pl(p{i}),i) = inv(Dl(pl(p{i}),:,i)/R(:,:,i)*Dl(pl(p{i}),:,i).');
    end
    
    for i = 1:size(t,2)

        E(:,:,i) = Cl(:,:,i) - Dl(:,:,i)/R(:,:,i)*K(:,:,i).';
        rh(:,i) = rl(:,i) - Dl(:,:,i)/R(:,:,i)*us0(:,i);
        
        Ah(:,:,i) = A(:,:,i) - B(:,:,i)/R(:,:,i)*(K(:,:,i).' + Dl(:,:,i).'*Dh(:,:,i)*E(:,:,i));
        Rh(:,:,i) = B(:,:,i)/R(:,:,i)*(eye(numInputs,numInputs)-Dl(:,:,i).'*Dh(:,:,i)*Dl(:,:,i)/R(:,:,i))*B(:,:,i).';
        Qh(:,:,i) = Q(:,:,i) - K(:,:,i)/R(:,:,i)*K(:,:,i).' + E(:,:,i).'*Dh(:,:,i)*E(:,:,i);
        x0h(:,i) = xs0(:,i) - K(:,:,i)/R(:,:,i)*us0(:,i) + E(:,:,i).'*Dh(:,:,i)*rh(:,i);
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
    nuV(:) = F\(-Gamma*xs(:,1) - y - H);
    
    nu = zeros(numActivePSConstraints,size(t,2));
    for i = 1:size(tk,2)
        nu(:,tk(i)) = nuV((i-1)*numActivePSConstraints+1:i*numActivePSConstraints);
    end   
    
    s = zeros(numStates,size(t,2));
    if numActivePSConstraints
        for i = 1:size(t,2)
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
        
    v = zeros(numStates,size(t,2));
    lambda = zeros(numStates,size(t,2));
    mu = zeros(numActiveSIConstraints,size(t,2));

    % Solve forward
    for i = 1:size(t,2)-1
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
        rhoi = ones(numStateInputConstraints,size(t,2));
        deltai = ones(numStateInputConstraints,size(t,2));
        
        rhoj = ones(numPureStateConstraints,size(t,2));
        deltaj = ones(numPureStateConstraints,size(t,2));

        for n = 1:size(t,2)
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
              
        thetak = min(-rhoi(~I & deltai>0)./deltai(~I & deltai>0));
        betak = min(-rhoj(~J & deltaj>0)./deltaj(~J & deltaj>0));

        alfak = min([1 thetak betak]);
        
        % Update controller
        x = x + alfak*xs;
        u = u + alfak*us;
        
        if alfak == 1
            step3 = true;
        else
            step3 = false;
            for n = 1:size(t,2)
                for i = 1:numStateInputConstraints
                    if(-rhoi(i,n)/deltai(i,n) == alfak && ~I(i,n) && n < size(t,2))
                        I(i,n) = 1;
                    end
                end
                for j = 1:numPureStateConstraints
                    if(-rhoj(j,n)/deltaj(j,n) == alfak && ~J(j,n)&& n > 1)                        
                        J(j,n) = 1;
                    end
                end
            end
        end
    end
    if step3        
        % Step 3
        if size(tl,2) > 0
            minMu = zeros(1,size(t,2));
            iS = zeros(1,size(t,2));

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
            minNu = zeros(1,size(t,2));
            jS = zeros(1,size(t,2));

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
        
        if minimumMu >= -1e-5 && minimumNu >=-1e-5 && norm(us)<=0.0001*norm(u)
            x = x + xs;
            u = u + us;
            break;
        else
            if minimumMu >= minimumNu && size(p{mS},1) > 0
                I(p{mS}(iS(mS)),mS) = 0;
            elseif minimumMu < minimumNu && size(q{lS},1) > 0
                J(q{lS}(jS(lS)),lS) = 0;
            end
        end
    end
    
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
hold off

figure(2)
plot(t,x(2,:))
title('Mass speed evolution','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('v(m/s)','interpreter','latex')
hold on
plot(tf,vf,'Marker','o','MarkerFaceColor','red')
hold off

figure(3)
plot(t,u)
title('Actuating force (u)','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('F(N)','interpreter','latex')
hold on
hold off


