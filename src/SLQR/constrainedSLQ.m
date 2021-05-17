function [x, u, I, J, converged] = constrainedSLQ(varargin)
%CONSTRAINEDSLQ Solves the constrained SLQR problem of the given system
%   Given the system modelled by "stateSpaceModel", the quadratic cost
%   function given in "costFunction" and the configurations given in
%   "config", one iteration of the SLQR constrained problem is solved,
%   trying to reach the objective given by "x0" and "u0". 
%   The results are returned in the given matrices "x" and "u", 
%   and the active constraints matrices "I" and "J" are also updated. 
%   Info about the map can be provided through "map" parameter.
% 
%   USAGE: 
%   [x, u, I, J, converged] = constrainedSLQ(x, x0, u, u0, dt,...
%                                         stateSpaceModel, costFunction,...
%                                         config)
%   [x, u, I, J, converged] = constrainedSLQ(x, x0, u, u0, dt,...
%                                         stateSpaceModel, costFunction,...
%                                         config, map)
%   [x, u, I, J, converged] = constrainedSLQ(x, x0, xs, u, u0, us, dt,...
%                                         stateSpaceModel, costFunction,...
%                                         config)
%   [x, u, I, J, converged] = constrainedSLQ(x, x0, xs, u, u0, us, dt,...
%                                         stateSpaceModel, costFunction,...
%                                         config, map)
% 
%   "x", "x0" are the states and goal states respectively. Size
%   numberStates x numberTimeSteps.
%
%   "u", "u0" are the control inputs and goal inputs respectively. Size
%   numberInputs x numberTimeSteps.
% 
%   "dt" is the time step.
%
%   "stateSpaceModel" should contain:
%       - Dynamic matrices "A", "B".
%       - State input constraints matrices "C", "D", "r".
%       - Active state input constraints matrices "I", "I0".
%       - Pure state constraints matrices "G", "h".
%       - Active pure state constraints matrices "J", "J0".
%       - Indexes of the XY pose of the robot "XYIndexes" (only needed if
%       checking safety w.r.t an obstacles map).
%
%   "costFunction" should contain:
%       - Pure state cost matrix "Q".
%       - Pure input cost matrix "R".
%       - State input cost matrix "K".
% 
%   "config" should contain:
%       - Acceptable distance to goal to consider convergence is reached,
%         "distThreshold". Default: 0.005.
%       - Acceptable control actuation step percentage to consider 
%         convergence is reached, "controlThreshold". Default: 1e-3.
%       - Percentage of constained timesteps for resampling
%         "resamplingThreshold". Default: 30.
%       - Step of the linear search procedure, "lineSearchStep". 
%         Default: 0.30.
%       - Yes/no about check distance to goal, "checkDistance".
%       - If checking distance to goal, indexes of state vector where
%         to check for the distance, "distIndexes".
%       - Yes/no about check constraints, "checkConstraints".
%       - Yes/no about check obstacles collisions, "checkSafety".
%
%   "map" should contain:
%       - Map resolution "mapResolution".
%       - Obstacles map "obstMap".
%       - X gradient of the obstacles map "gradientObstaclesMapX".
%       - Y gradient of the obstacles map "gradientObstaclesMapY".
%       - Repulsive cost from obstacles "obstaclesCost".
%
%   If convergence is reached "converged" will return "1", if not:
%       " 0" --> the algorithm should continue iterating.
%       "-1" --> the goal is still far away.
%       "-2" --> the algorithm is still hardly updating the control.
%       "-3" --> the generated state is not safe.
%       "-4" --> the imposed constraints are not complied.
%       "-5" --> something unexpected happened.

    
    switch nargin
        case 8
            x = varargin{1};
            x0 = varargin{2};
            xs = zeros(size(x));
            u = varargin{3};
            u0 = varargin{4};
            us = zeros(size(u));
            dt = varargin{5};
            stateSpaceModel = varargin{6};
            costFunction = varargin{7};
            config = varargin{8};
        case 9
            x = varargin{1};
            x0 = varargin{2};
            xs = zeros(size(x));
            u = varargin{3};
            u0 = varargin{4};
            us = zeros(size(u));
            dt = varargin{5};
            stateSpaceModel = varargin{6};
            costFunction = varargin{7};
            config = varargin{8};
            map = varargin{9};
        case 10
            x = varargin{1};
            x0 = varargin{2};
            xs = varargin{3};
            u = varargin{4};
            u0 = varargin{5};
            us = varargin{6};
            dt = varargin{7};
            stateSpaceModel = varargin{8};
            costFunction = varargin{9};
            config = varargin{10};
        case 11
            x = varargin{1};
            x0 = varargin{2};
            xs = varargin{3};
            u = varargin{4};
            u0 = varargin{5};
            us = varargin{6};
            dt = varargin{7};
            stateSpaceModel = varargin{8};
            costFunction = varargin{9};
            config = varargin{10};
            map = varargin{11};
        otherwise
            cprintf('err','Wrong number of inputs. Usage:\n')
            cprintf('err','    constrainedSLQ(x, x0, u, u0, dt, stateSpaceModel, costFunction, config)\n')
            cprintf('err','    constrainedSLQ(x, x0, u, u0, dt, stateSpaceModel, costFunction, config, map)\n')
            cprintf('err','    constrainedSLQ(x, x0, xs, u, u0, us, dt, stateSpaceModel, costFunction, config)\n')
            cprintf('err','    constrainedSLQ(x, x0, xs, u, u0, us, dt, stateSpaceModel, costFunction, config, map)\n')

            error('Too many input arguments.');
    end

    % Extracting the state space model
    A = stateSpaceModel.A;
    B = stateSpaceModel.B;
    C = stateSpaceModel.C;
    D = stateSpaceModel.D;
    r = stateSpaceModel.r;
    G = stateSpaceModel.G;
    h = stateSpaceModel.h;
    I = stateSpaceModel.I;
    J = stateSpaceModel.J;
    I0 = stateSpaceModel.I0;
    J0 = stateSpaceModel.J0;
    
    % Extracting the quadratized cost function
    Q = costFunction.Q;
    R = costFunction.R;
    K = costFunction.K;
    
    % Extracting SLQ configuration
    distThreshold = config.distThreshold;
    resamplingThreshold = config.resamplingThreshold;
    lineSearchStep = config.lineSearchStep;
    controlThreshold = config.controlThreshold;
    checkingDistance = config.checkDistance;
    if checkingDistance
        distIndexes = config.distIndexes;
    end

    checkingConstraints = config.checkConstraints;
    checkingSafety = config.checkSafety;

    % Extracting map info
    mapResolution = [];
    obstMap = [];
    if checkingSafety
        mapResolution = map.mapResolution;
        obstMap = map.obstMap;
        robotXIndex = map.XYIndexes(1);
        robotYIndex = map.XYIndexes(2);
        gradientOMX = map.gradientObstaclesMapX;
        gradientOMY = map.gradientObstaclesMapY;
        obstaclesCost = map.obstaclesCost;
        robotXIndex = stateSpaceModel.XYIndexes(1);
        robotYIndex = stateSpaceModel.XYIndexes(2);
    end

    % Model characteristics
    numStates = size(x,1);
    numInputs = size(u,1);
    timeSteps = size(x,2);
    numStateInputConstraints = size(I,1);
    numPureStateConstraints = size(J,1);
    
    % Variable to check if the algorithm has already converged
    converged = 0;    
        
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
        
    % Obstacles limits cost
    Ox = zeros(numStates,timeSteps);
    if checkingSafety
        for i = 1:timeSteps-1
            [Ox(robotXIndex,i), Ox(robotYIndex,i)] = ...
              getGradientTotalCost(x(robotXIndex,i),...
                                   x(robotYIndex,i),...
                                   mapResolution,...
                                   gradientOMX, gradientOMY);
            Ox(robotXIndex,i) = obstaclesCost*Ox(robotXIndex,i);
            Ox(robotYIndex,i) = obstaclesCost*Ox(robotYIndex,i);
        end
    end
    
    % LQ problem solution
    M = zeros(numStates,numStates,timeSteps);
    P = zeros(numStates,numStates,timeSteps);
    z = zeros(numStates,timeSteps);
    P(:,:,end) = Q(:,:,end);
    z(:,end) = xs0(:,end) + Ox(:,end);
    
    % Solve backward
    for i = timeSteps-1:-1:1
        M(:,:,i) = inv(eye(numStates) + Rh(:,:,i)*P(:,:,i+1));
        P(:,:,i) = Qh(:,:,i) + Ah(:,:,i).'*P(:,:,i+1)*M(:,:,i)*Ah(:,:,i);
        z(:,i) = Ah(:,:,i).'*M(:,:,i).'*z(:,i+1) + ...
            Ah(:,:,i).'*P(:,:,i+1)*M(:,:,i)*u0h(:,i) + x0h(:,i) + Ox(:,i);
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
        
    step3 = true;
    if norm(us)>=controlThreshold*norm(u)
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

%             xk = x;
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
            [~, ind] = min(Jcost);
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
%                 if sum(I(i,:)) > timeSteps*resamplingThreshold/100
%                     I(i,:) = 0;
%                     disp('Resampling...');
%                 end
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
        convergenceCondition = norm(us) <= controlThreshold*norm(u);
        if checkingDistance
            endDist = norm(x(distIndexes,end)-x0(distIndexes,end));
            convergenceCondition = convergenceCondition | ...
                                (norm(us) <= controlThreshold*20*norm(u) & ...
                                endDist < distThreshold);
        end
        if checkingSafety
            convergenceCondition = convergenceCondition & ...
                isSafePath(x(robotXIndex,:),x(robotYIndex,:), mapResolution,obstMap);
        end
        if checkingConstraints
            convergenceCondition = convergenceCondition & ...
                   checkConstraints(x, u, stateSpaceModel);
        end

        %if minimumMu >= -1e-5 && minimumNu >=-1e-5 && ...
        if convergenceCondition         
%             u = u + us;            
            x = forwardIntegrateSystem(x, u, dt);
            converged = 1;
        else
            if minimumMu <= minimumNu && size(p{mS},1) > 0
                I(p{mS}(iS(mS)),mS) = 0;
            elseif minimumMu > minimumNu && size(q{lS},1) > 0
                J(q{lS}(jS(lS)),lS) = 0;
            end
            
            if checkingDistance
                if endDist > distThreshold
                    converged = -1;
                end
            elseif norm(us) > controlThreshold*norm(u)
                converged = -2;
            elseif checkingSafety
                if ~isSafePath(x(robotXIndex,:),x(robotYIndex,:), mapResolution,obstMap)
                    converged = -3;
                end
            elseif checkingConstraints
                if ~checkConstraints(x, u, stateSpaceModel)
                    converged = -4;
                end
            else
                converged = -5;
            end
        end
    end
end

