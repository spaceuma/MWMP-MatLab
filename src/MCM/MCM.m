function x0 = MCM(x, x0, map, stateSpaceModel,...
                  trajectoryInfo, config)
%MCM Multitrajectory Costing Method
%   A new reference paths is built in function of the current path of
%   the platform "x" and its cost w.r.t. the current reference path "x0".
% 
%   "x", "x0" are the states and goal states respectively. Size
%   numberStates x numberTimeSteps.
% 
%   "map" defines the characteristics of the scenario, should contain:
%       - Map resolution "mapResolution".
%       - Obstacles map "obstMap".
%       - X gradient of the total cost map "gradientTotalCostMapX".
%       - Y gradient of the total cost map "gradientTotalCostMapY".
%       - Index of the goal inside the map "iGoal".
% 
%   "stateSpaceModel" defines the indexes of different features:
%       - Indexes of the XY pose of the robot "XYIndexes".
%       - Index of the Theta yaw orientation of the platform "thetaIndex".
% 
%   "trajectoryInfo" defines characteristics of the trajectories:
%       - Percentage of map resolution used to generate trajectories "tau".
%       - Separation in meters between consecutive waypoints
%         "waypointSeparation".

    % Extracting map info
    mapResolution = map.mapResolution;
    obstMap = map.obstMap;
    gradientTCMX = map.gradientTotalCostMapX;
    gradientTCMY = map.gradientTotalCostMapY;
    totalCostMap = map.totalCostMap;
    iGoal = map.iGoal;

    % Extracting state space model info
    robotXIndex = stateSpaceModel.XYIndexes(1);
    robotYIndex = stateSpaceModel.XYIndexes(2);
    robotThetaIndex = stateSpaceModel.thetaIndex;
    
    % Extracting trajectory info
    tau = trajectoryInfo.tau;
    waypSeparation = trajectoryInfo.waypointSeparation;

    % Extracting configuration of the algorithm     
    maxDistUpdate = config.maxDistUpdate;
    minDistUpdate = config.minDistUpdate;
    costThreshold = config.costThreshold;
    percentageMCM = config.percentageMCM;

    % Model characteristics
    timeSteps = size(x,2);

    % Filtering undesired updates
    for i = 2:ceil(timeSteps*(100-percentageMCM)/100):timeSteps-2
        d = norm(x([robotXIndex robotYIndex],i) - ...
                 x0([robotXIndex robotYIndex],i));
        if d < maxDistUpdate && ...
           d > minDistUpdate && ...
           isSafePath(x(robotXIndex,:), x(robotYIndex,:), ...
                      mapResolution,obstMap)
            % Obtaining the candidate path
            iInit = [round(x(robotXIndex,i)/mapResolution)+1 ...
                     round(x(robotYIndex,i)/mapResolution)+1];
            if(iInit(1) > size(totalCostMap,1)-2)
                iInit(1) = size(totalCostMap,1)-2;
            end
            if(iInit(1) < 3)
                iInit(1) = 3;
            end
            if(iInit(2) > size(totalCostMap,2)-2)
                iInit(2) = size(totalCostMap,2)-2;
            end
            if(iInit(2)<3)
                iInit(2) = 3;
            end
            [pathi,~] = getPathGDM(totalCostMap, iInit, iGoal, tau, ...
                                   gradientTCMX, gradientTCMY);
            pathi = (pathi-1)*mapResolution;

            while(size(pathi,1) > 1000)
                [pathi,~] = getPathGDM(totalCostMap,...
                                       iInit + round(2*rand(1,2)-1),...
                                       iGoal, tau, ...
                                       gradientTCMX, gradientTCMY);
                pathi = (pathi-1)*mapResolution;
            end
            

            yaw = getYaw(pathi);
            pathi = [pathi yaw].';
            
            % Compute switch waypoint
            [~, switchIndex] = getSwitch(x0([robotXIndex...
                                             robotYIndex...
                                             robotThetaIndex],:),...
                                             pathi, i);
            % The above option is computionally more expensive
%             switchIndex = i;
%             switchPose = x0(1:2,i);

            % Compute intersection waypoint
            [~, interIndex1, interIndex2] = ...
                getIntesection(pathi, x0([robotXIndex...
                                          robotYIndex...
                                          robotThetaIndex],...
                                          switchIndex:end), ...
                                          waypSeparation);
            interIndex2 = interIndex2 + switchIndex - 1;
                        
            % If the candidate path is big enough...
            if(interIndex1 > 1 && interIndex2 ~= switchIndex)
                pathCandidate = pathi(:,1:interIndex1);
                originalPath = x0([robotXIndex...
                                   robotYIndex...
                                   robotThetaIndex],...
                                   switchIndex:interIndex2);

                % Resize path
                x1 = 1:size(pathCandidate,2);
                x2 = linspace(1,size(pathCandidate,2),...
                                size(originalPath,2));
                pathCandidate = (interp1(x1,pathCandidate.',x2)).';

                % Get path costs (candidate and original)
                newCost = getTrajectoryCost(pathCandidate, ...
                                            x([robotXIndex ...
                                               robotYIndex ...
                                               robotThetaIndex],:), ...
                                               switchIndex);
                                           
                oldCost = getTrajectoryCost(originalPath, ...
                                            x([robotXIndex ...
                                               robotYIndex ...
                                               robotThetaIndex],:), ...
                                               switchIndex);

                % If the cost is reduced, the candidate is safe and the
                % change is significant, update the reference path
                if newCost*(1 + costThreshold/100) < oldCost && ...
                   isSafePath([x(robotXIndex,1:switchIndex-1) ...
                               pathCandidate(1,:) ...
                               x(robotXIndex,interIndex2+1:end)],...
                              [x(robotYIndex,1:switchIndex-1) ...
                               pathCandidate(2,:) ...
                               x(robotYIndex,interIndex2+1:end)],...
                               mapResolution,obstMap) && ...
                              DiscreteFrechetDist(pathCandidate, ...
                                                  originalPath) > ...
                                                  2*waypSeparation
                                              
                    x0([robotXIndex ...
                        robotYIndex ...
                        robotThetaIndex], ...
                        switchIndex:interIndex2) = pathCandidate;
                    
                    for j = switchIndex:interIndex2
                        if j > 1
                            x0(robotThetaIndex,j) = ...
                                modulateYaw(x0(robotThetaIndex,j), ...
                                            x0(robotThetaIndex,j-1));
                        end
                    end
                    
                    disp(['Changing reference path from waypoint ',...
                          num2str(switchIndex),...
                          ' to ',num2str(interIndex2)])
                end
            end
        end
    end  
end

