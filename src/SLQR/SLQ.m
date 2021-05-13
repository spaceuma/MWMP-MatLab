function [x, u, converged] = SLQ(x, x0, u, u0, dt,...
                                 stateSpaceModel, costFunction, config)
%SLQ Solves the SLQR problem of the given system
%   Given the system modelled by "stateSpaceModel", the quadratic cost
%   function given in "costFunction" and the configurations given in
%   "config", one iteration of the SLQR problem is solved,
%   trying to reach the objective given by "x0" and "u0". 
%   The results are returned in the given matrices "x" and "u".
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
%       - Dynamic matrices A, B.
%
%   "costFunction" should contain:
%       - Pure state cost matrix Q.
%       - Pure input cost matrix R.
% 
%   "config" should contain:
%       - Acceptable distance to goal to consider convergence is reached,
%         "distThreshold". Default: 0.005.
%       - Acceptable control actuation step percentage to consider 
%         convergence is reached, "controlThreshold". Default: 1e-3.
%       - Step of the linear search procedure, "lineSearchStep". 
%         Default: 0.30.
%
%   If convergence is reached "converged" will return "1", if not:
%       " 0" --> something unexpected happened.
%       "-1" --> the goal is still far away.
%       "-2" --> the algorithm is still hardly updating the control.
%       "-3" --> something unexpected happened.
    
    % Extracting the state space model
    A = stateSpaceModel.A;
    B = stateSpaceModel.B;
    
    % Extracting the quadratized cost function
    Q = costFunction.Q;
    R = costFunction.R;
    
    % Extracting SLQ configuration
    distThreshold = config.distThreshold;
    lineSearchStep = config.lineSearchStep;
    controlThreshold = config.controlThreshold;

    % Model characteristics
    numStates = size(x,1);
    numInputs = size(u,1);
    timeSteps = size(x,2);
    
    % Variable to check if the algorithm has already converged
    converged = 0;    
    
    % Update reference trajectories    
    xh0 = x0 - x;
    uh0 = u0 - u;    
    
    % LQ problem solution
    M = zeros(numStates,numStates,timeSteps);
    P = zeros(numStates,numStates,timeSteps);
    s = zeros(numStates,1,timeSteps);
    
    P(:,:,end) = Q(:,:,end);
    s(:,:,end) = -Q(:,:,end)*xh0(:,end);
    
    xh = zeros(numStates,timeSteps);
    uh = zeros(numInputs,timeSteps);
    v = zeros(size(xh));
    lambdah = zeros(size(s));
        
    % Solve backward
    for i = timeSteps-1:-1:1
        M(:,:,i) = inv(eye(numStates) + B(:,:,i)/R*B(:,:,i).'*P(:,:,i+1));
        P(:,:,i) = Q(:,:,i) + A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*A(:,:,i);
        s(:,:,i) = A(:,:,i).'*(eye(size(Q,1)) - ...
                   P(:,:,i+1)*M(:,:,i)*B(:,:,i)/R*B(:,:,i).')*s(:,:,i+1)+...
                   A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*B(:,:,i)*uh0(:,i) -...
                   Q(:,:,i)*xh0(:,i);
    end
    
    % Solve forward
    for i = 1:timeSteps-1
        v(:,i) = M(:,:,i)*B(:,:,i)*(uh0(:,i)-R\B(:,:,i).'*s(:,:,i+1));
        xh(:,i+1) = M(:,:,i)*A(:,:,i)*xh(:,i)+v(:,i);
        lambdah(:,:,i+1) = P(:,:,i+1)*xh(:,i+1)+s(:,:,i+1);
        uh(:,i) = uh0(:,i)-R\B(:,:,i).'*lambdah(:,:,i+1);
    end
        
    % Exit condition
    endDist = norm(x(1:3,end)-x0(1:3,end));
    
    if norm(uh) <= controlThreshold*norm(u) ||...
       norm(uh) <= controlThreshold*20*norm(u) &&...
       endDist < distThreshold
        converged = 1;
    else
        if endDist > distThreshold
            converged = -1;
        elseif norm(us) > controlThreshold*norm(u)
            converged = -2;
        else
            converged = -3;
        end
            
        % Line search to optimize alfa
        alfa = 1:-lineSearchStep:0.0001;
        J = zeros(size(alfa));
        uk = u;
        for n = 1:size(alfa,2)
            u = uk + alfa(n)*uh;
            x = forwardIntegrateSystem(x, u, dt);

            J(n) = 1/2*(x(:,end)-x0(:,end)).'*Q(:,:,end)*(x(:,end)-x0(:,end));
            for i = 1:timeSteps-1
                J(n) = J(n) + 1/2*((x(:,i)-x0(:,i)).'*Q(:,:,i)*(x(:,i)-x0(:,i))...
                    + (u(:,i)-u0(:,i)).'*R*(u(:,i)-u0(:,i)));
            end            
        end
        [~, ind] = min(J);
        alfamin = alfa(ind);
        
        % Update controller
        u = uk + alfamin*uh;
        x = forwardIntegrateSystem(x, u, dt);
    end                
end
