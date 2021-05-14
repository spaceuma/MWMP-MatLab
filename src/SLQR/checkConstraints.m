function constraintsSatisfied = checkConstraints(x, u, stateSpaceModel)
%CHECKCONSTRAINTS Check if the system is satisfying the specified
%constraints
%   Given the constraints matrices C, D, r, G and h in "stateSpaceModel",
%   the state "x" and the control input "u", this functions returns a 0 
%   if the constraints are not satisfied or a 1 if the contraints are
%   satisfied

    % Extracting the state space model
    C = stateSpaceModel.C;
    D = stateSpaceModel.D;
    r = stateSpaceModel.r;
    G = stateSpaceModel.G;
    h = stateSpaceModel.h;
    
    % Model characteristics
    numStateInputConstraints = size(C,1);
    numPureStateConstraints = size(G,1);
    timeSteps = size(x,2);

    % Checking constraints
    constraintsSatisfied = 1;   
    for n = 1:timeSteps
        for i = 1:numStateInputConstraints
            rhoi = C(i,:,n)*x(:,n) + D(i,:,n)*u(:,n) + r(i,n);
            if rhoi > 1e-4
                constraintsSatisfied = 0;
                return;
            end
        end
        for j = 1:numPureStateConstraints
            rhoj = G(j,:,n)*x(:,n) + h(j,n);
            if rhoj > 1e-4
                constraintsSatisfied = 0;
                return;
            end
        end
    end
end

