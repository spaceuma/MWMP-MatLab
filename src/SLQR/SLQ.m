function [x, u, converged] = SLQ(varargin)
%SLQ Solves the SLQR problem of the given system
%   Given the system modelled by "stateSpaceModel", the quadratic cost
%   function given in "costFunction" and the configurations given in
%   "config", one iteration of the SLQR problem is solved,
%   trying to reach the objective given by "x0" and "u0". 
%   The results are returned in the given matrices "x" and "u".
%   Info about the map can be provided through "map" parameter.
%
%   USAGE: 
%   [x, u, converged] = SLQ(x, x0, u, u0, dt,...
%                           stateSpaceModel, costFunction, config)
%   [x, u, converged] = SLQ(x, x0, u, u0, dt,...
%                           stateSpaceModel, costFunction, config, map)
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
%       - Indexes of the XY pose of the robot "XYIndexes" (only needed if
%       checking safety w.r.t an obstacles map).
% 
%   "costFunction" should contain:
%       - Pure state cost matrix "Q".
%       - Pure input cost matrix "R".
% 
%   "config" should contain:
%       - Acceptable control actuation step percentage to consider 
%         convergence is reached, "controlThreshold". Default: 1e-3.
%       - Step of the linear search procedure, "lineSearchStep". 
%         Default: 0.30.
%       - Yes/no about check distance to goal, "checkDistance".
%       - If checking distance to goal, indexes of state vector where
%         to check for the distance, "distIndexes".
%       - If checking distance to goal, acceptable distance to goal 
%         to consider convergence is reached, "distThreshold". 
%         Default: 0.005.
%       - Yes/no about check final orientation, "checkOrientation".
%       - If checking final orientation, indexes of state vector where
%         to check for the orientation, "orientationIndexes".
%       - If checking final orientation, acceptable euclidean distance to 
%         the final orientation to consider convergence is reached,
%         "orientationThreshold". Default: 0.005.
%       - Yes/no about check obstacles collisions, "checkSafety".
%
%   "map" only needed if checking safety, should contain:
%       - Map resolution "mapResolution".
%       - Obstacles map "obstMap".
%       - X gradient of the obstacles map "gradientObstaclesMapX".
%       - Y gradient of the obstacles map "gradientObstaclesMapY".
%       - Repulsive cost from obstacles "obstaclesCost".
% 
%   If convergence is reached "converged" will return "1", if not:
%       " 0" --> something unexpected happened.
%       "-1" --> the goal is still far away.
%       "-2" --> the algorithm is still hardly updating the control.
%       "-3" --> the generated state is not safe.
%       "-4" --> something unexpected happened.

    switch nargin
        case 8
            x = varargin{1};
            x0 = varargin{2};
            u = varargin{3};
            u0 = varargin{4};
            dt = varargin{5};
            stateSpaceModel = varargin{6};
            costFunction = varargin{7};
            config = varargin{8};
        case 9
            x = varargin{1};
            x0 = varargin{2};
            u = varargin{3};
            u0 = varargin{4};
            dt = varargin{5};
            stateSpaceModel = varargin{6};
            costFunction = varargin{7};
            config = varargin{8};
            map = varargin{9};
        otherwise
            cprintf('err','Wrong number of inputs. Usage:\n')
            cprintf('err','    SLQ(x, x0, u, u0, dt, stateSpaceModel, costFunction, config)\n')
            cprintf('err','    SLQ(x, x0, u, u0, dt, stateSpaceModel, costFunction, config, map)\n')
            error('Too many input arguments.');
    end    

    % Extracting the state space model
    A = stateSpaceModel.A;
    B = stateSpaceModel.B;
    
    % Extracting the quadratized cost function
    Q = costFunction.Q;
    R = costFunction.R;
    
    % Extracting SLQ configuration
    lineSearchStep = config.lineSearchStep;
    controlThreshold = config.controlThreshold;
    checkingDistance = config.checkDistance;
    if checkingDistance
        distIndexes = config.distIndexes;
        distThreshold = config.distThreshold;
    end
    checkingOrientation = config.checkOrientation;
    if checkingOrientation
        orientationIndexes = config.orientationIndexes;
        orientationThreshold = config.orientationThreshold;
    end    
    checkingSafety = config.checkSafety;
    
    % Extracting map info
    mapResolution = [];
    obstMap = [];
    if checkingSafety
        mapResolution = map.mapResolution;
        obstMap = map.obstMap;
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
    
    % Variable to check if the algorithm has already converged
    converged = 0;    
    
    % Update reference trajectories    
    xh0 = x0 - x;
    uh0 = u0 - u;    
    
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
    s = zeros(numStates,1,timeSteps);
    
    P(:,:,end) = Q(:,:,end);
    s(:,:,end) = -Q(:,:,end)*xh0(:,end) + Ox(:,end);
    
    xh = zeros(numStates,timeSteps);
    uh = zeros(numInputs,timeSteps);
    v = zeros(size(xh));
    lambdah = zeros(size(s));
        
    % Solve backward
    for i = timeSteps-1:-1:1
        try
            M(:,:,i) = inv(eye(numStates) + B(:,:,i)/R(:,:,i)*B(:,:,i).'*P(:,:,i+1));
        catch
            M(:,:,i) = pinv(eye(numStates) + B(:,:,i)/R(:,:,i)*B(:,:,i).'*P(:,:,i+1));
        end
        P(:,:,i) = Q(:,:,i) + A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*A(:,:,i);
        s(:,:,i) = A(:,:,i).'*(eye(size(Q,1)) - ...
                   P(:,:,i+1)*M(:,:,i)*B(:,:,i)/R(:,:,i)*B(:,:,i).')*s(:,:,i+1)+...
                   A(:,:,i).'*P(:,:,i+1)*M(:,:,i)*B(:,:,i)*uh0(:,i) -...
                   Q(:,:,i)*xh0(:,i) + Ox(:,i);
    end
    
    % Solve forward
    for i = 1:timeSteps-1
        v(:,i) = M(:,:,i)*B(:,:,i)*(uh0(:,i)-R(:,:,i)\B(:,:,i).'*s(:,:,i+1));
        xh(:,i+1) = M(:,:,i)*A(:,:,i)*xh(:,i)+v(:,i);
        lambdah(:,:,i+1) = P(:,:,i+1)*xh(:,i+1)+s(:,:,i+1);
        uh(:,i) = uh0(:,i)-R(:,:,i)\B(:,:,i).'*lambdah(:,:,i+1);
    end
        
    % Exit condition
    convergenceCondition = norm(uh) <= controlThreshold*norm(u);
    if checkingDistance
        endDist = norm(x(distIndexes,end)-x0(distIndexes,end));
        convergenceCondition = convergenceCondition  & endDist < distThreshold | ...
                            (norm(uh) <= controlThreshold*20*norm(u) & ...
                            endDist < distThreshold);
    end
    if checkingOrientation
        endOrientation = norm(x(orientationIndexes,end)-x0(orientationIndexes,end));
        convergenceCondition = convergenceCondition & ...
                               endOrientation < orientationThreshold;
    end
    if checkingSafety
        convergenceCondition = convergenceCondition & ...
               isSafePath(x(robotXIndex,:),x(robotYIndex,:), mapResolution,obstMap);
    end
    
    if convergenceCondition
        converged = 1;
    else
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
        else
            converged = -4;
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
                    + (u(:,i)-u0(:,i)).'*R(:,:,i)*(u(:,i)-u0(:,i)));
            end            
        end
        [~, ind] = min(J);
        alfamin = alfa(ind);
        
        % Update controller
        u = uk + alfamin*uh;
        x = forwardIntegrateSystem(x, u, dt);
    end                
end
