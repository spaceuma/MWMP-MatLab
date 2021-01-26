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
yf = 0.05;
vf = 0;

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
    
    Q(:,:,end) = [0 0;
                  0 0];
        
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
    
    % Constraints definition
    % State input constraints
    numStateInputConstraints = 1;
    tl = zeros(numStateInputConstraints);
    tl(1) = 100;
    p = zeros(size(t));
    p(tl(1)) = 1;

    C = zeros(max(p),numStates,size(t,2));
    D = zeros(max(p),numInputs,size(t,2));
    r = zeros(max(p),size(t,2));
    
    C(1,:,tl(1)) = zeros(1,numStates);
    D(1,:,tl(1)) = 1;
    r(1,tl(1)) = -1;
    
    % Pure state constraints
    numPureStateConstraints = 3;
    tk = zeros(numPureStateConstraints);
    tk(1) = 50;
    tk(2) = 250;
    tk(3) = size(t,2);
    
    q = zeros(size(t));
    q(tk(1)) = 1;
    q(tk(2)) = 1;
    q(tk(3)) = 2;
    
    G = zeros(max(q),numStates,size(t,2));
    h = zeros(max(q),size(t,2));
    
    G(1,1,tk(1)) = 1;
    h(1,tk(1)) = -0.1;

    G(1,2,tk(2)) = 1;
    h(1,tk(2)) = -0.1;
    
    G(1,1,tk(3)) = 1;
    h(1,tk(3)) = -yf;
    G(2,2,tk(3)) = 1;
    h(2,tk(3)) = -vf;
    
    % Predefinitions
    Dh = zeros(max(p),max(p),size(t,2));
    E = zeros(max(p),numStates,size(t,2));
    rh = zeros(max(p),size(t,2));
    
    Ah = zeros(numStates,numStates,size(t,2));
    Rh = zeros(numStates,numStates,size(t,2));
    Qh = zeros(numStates,numStates,size(t,2));
    x0h = zeros(numStates,size(t,2));
    u0h = zeros(numStates,size(t,2));
    
    
    for i = 1:size(t,2)
        if(p(i))
            Dh(:,:,i) = inv(D(:,:,i)/R(:,:,i)*D(:,:,i).');
        end
        E(:,:,i) = C(:,:,i);
        rh(:,i) = r(:,i) - D(:,:,i)/R(:,:,i)*us0(:,i);
        
        Ah(:,:,i) = A(:,:,i) - B(:,:,i)/R(:,:,i)*D(:,:,i).'*Dh(:,:,i)*E(:,:,i);
        Rh(:,:,i) = B(:,:,i)/R(:,:,i)*(eye(numInputs,numInputs)-D(:,:,i).'*Dh(:,:,i)*D(:,:,i)/R(:,:,i))*B(:,:,i).';
        Qh(:,:,i) = Q(:,:,i) + E(:,:,i).'*Dh(:,:,i)*E(:,:,i);
        x0h(:,i) = xs0(:,i) + E(:,:,i).'*Dh(:,:,i)*rh(:,i);
        u0h(:,i) = -B(:,:,i)/R(:,:,i)*(us0(:,i) + D(:,:,i).'*Dh(:,:,i)*rh(:,i));
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
    
    Gamma = zeros(max(q)*numPureStateConstraints,numStates);
    Gammak = zeros(max(q),numStates,size(t,2), numPureStateConstraints);
    y = zeros(max(q)*numPureStateConstraints,1);
    F = zeros(max(q)*numPureStateConstraints,max(q)*numPureStateConstraints);
    H = zeros(max(q)*numPureStateConstraints,1);

    % Solve over all state constraints   
    for k = 1:numPureStateConstraints
        yk = zeros(max(q),tk(k));

        Gammak(:,:,tk(k),k) = G(:,:,tk(k));

        for n = tk(k)-1:-1:1
            Gammak(:,:,n,k) = Gammak(:,:,n+1,k)*M(:,:,n)*Ah(:,:,n);
            yk(:,n) = yk(:,n+1) + Gammak(:,:,n+1,k)*M(:,:,n)*(u0h(:,n) - Rh(:,:,n) * z(:,n+1));
        end

        for j = 1:numPureStateConstraints
            minConstraintIndex = min(tk(k),tk(j));
            Fkj = zeros(max(q),max(q),minConstraintIndex);
            for n = minConstraintIndex-1:-1:1
                Fkj(:,:,n) = Fkj(:,:,n+1) - Gammak(:,:,n+1,k)*M(:,:,n)*Rh(:,:,n)*Gammak(:,:,n+1,j).';
            end
            F((k-1)*max(q)+1:k*max(q),(j-1)*max(q)+1:j*max(q)) = Fkj(:,:,1);
        end

        H((k-1)*max(q)+1:k*max(q)) = h(:,tk(k));
        Gamma((k-1)*max(q)+1:k*max(q),:) = Gammak(:,:,1,k);
        y((k-1)*max(q)+1:k*max(q)) = yk(:,1);        

    end
       
    nu = zeros(max(q)*numPureStateConstraints,1);
    invF = zeros(size(F));
    if(~det(F))
        correctIndex = [];
        currentIndex = 1;
        for k = 1:numPureStateConstraints
            tConstraint = tk(k);
            correctIndex = [correctIndex currentIndex:currentIndex+q(tConstraint)-1];
            currentIndex = currentIndex + max(q);            
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
        for k = 1:numPureStateConstraints
            if (tk(k) > i)
                sum = sum + Gammak(:,:,i,k).'*nu((k-1)*max(q)+1:k*max(q));
            end
        end
        s(:,i) = z(:,i) + sum;
    end
    
    v = zeros(numStates,size(t,2));
    lambda = zeros(numStates,size(t,2));
    mu = zeros(max(p),size(t,2));
    
%     xsk(:,1) = zeros(numStates,1);
%     xs(:,1) = xs0(:,1);

    % Solve forward
    for i = 1:size(t,2)-1
        v(:,i) = M(:,:,i)*(u0h(:,i) - Rh(:,:,i)*s(:,i+1));
        xs(:,i+1) = M(:,:,i)*Ah(:,:,i)*xs(:,i) + v(:,i);
        lambda(:,i+1) = P(:,:,i+1)*xs(:,i+1) + s(:,i+1);
        mu(:,i) = Dh(:,:,i)*(E(:,:,i)*xs(:,i) - D(:,:,i)/R(:,:,i)*B(:,:,i).'*lambda(:,i+1) + rh(:,i));
        us(:,i) = -R(:,:,i)\(B(:,:,i).'*lambda(:,i+1) + D(:,:,i).'*mu(:,i) + us0(:,i));
    end
   
    % Exit condition
    if norm(us)<=0.000001*norm(u) || iter >= 2
        iter = iter-1;
        break;
    else
%         % Line search to optimize alfa
%         alfa = 1:-lineSearchStep:0.0001;
%         J = zeros(size(alfa));
%         for n = 1:size(alfa,2)
%             xaux = x + alfa(n)*xs;
%             uaux = u + alfa(n)*us;
%             J(n) = 1/2*(xaux(:,end)-x0(:,end)).'*Q(:,:,end)*(xaux(:,end)-x0(:,end));
%             for i = 1:size(t,2)-1
%                 J(n) = J(n) + 1/2*((xaux(:,i)-x0(:,i)).'*Q(:,:,i)*(xaux(:,i)-x0(:,i))...
%                     + (uaux(:,i)-u0(:,i)).'*R*(uaux(:,i)-u0(:,i)));
%             end            
%         end
%         [mincost, ind] = min(J);
%         alfamin = alfa(ind);
%         
%         % Update controller
%         x = x + alfamin*xs;
%         u = u + alfamin*us;
        x = x + xs;
        u = u + us;

        iter = iter+1;
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
plot((tk(1)-1)*dt,-h(1,tk(1)),'Marker','o','MarkerFaceColor','red')
hold off

figure(2)
plot(t,x(2,:))
title('Mass speed evolution','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('v(m/s)','interpreter','latex')
hold on
plot(tf,vf,'Marker','o','MarkerFaceColor','red')
plot((tk(2)-1)*dt,-h(1,tk(2)),'Marker','o','MarkerFaceColor','red')
hold off

figure(3)
plot(t,u)
title('Actuating force (u)','interpreter','latex')
xlabel('t(s)','interpreter','latex')
ylabel('F(N)','interpreter','latex')
hold on
plot((tl(1)-1)*dt,-r(1,tl(1)),'Marker','o','MarkerFaceColor','red')
hold off


