function h = plotFrame(varargin)
%PLOTFRAME Plot a given frame
%   Plots the x, y, and z axes of the frame given by a
%   transform matrix "T" on a figure "f", with the length given by
%   axisLength (1 by default) and the name given by frameName.

    switch nargin
        case 0
            cprintf('err','No arguments provided to the function. Usage:\n')
            cprintf('err','    plotReferenceFrame(T)\n')
            cprintf('err','    plotReferenceFrame(T, figure)\n')
            cprintf('err','    plotReferenceFrame(T, figure, axisLength)\n')
            cprintf('err','    plotReferenceFrame(T, figure, axisLength, frameName)\n')
            error('Not enough input arguments');
        case 1
            T = varargin{1};
            f = 1;
            axisLength = 1;
            frameName = 0;
        case 2
            T = varargin{1};
            f = varargin{2};
            axisLength = 1;
            frameName = 0;

        case 3
            T = varargin{1};
            f = varargin{2};
            axisLength = varargin{3};
            frameName = 0;

        case 4
            T = varargin{1};
            f = varargin{2};
            axisLength = varargin{3};
            frameName = varargin{4};
            if ~(isstring(frameName)||ischar(frameName))
                error('Input name of the frame is not valid string or char');
            end
        otherwise
            cprintf('err','Too many arguments provided. Usage:\n')
            cprintf('err','    plotReferenceFrame(T)\n')
            cprintf('err','    plotReferenceFrame(T, figure)\n')
            cprintf('err','    plotReferenceFrame(T, figure, axisLength)\n')
            cprintf('err','    plotReferenceFrame(T, figure, axisLength, frameName)\n')
            error('Too many input arguments.');
    end
    
    if size(T) ~= [4 4]
        error('Input transformation matrix is not 4x4');
    end
    
    origin = T(1:3,4);
    xaxis = T(1:3,1)*axisLength;
    yaxis = T(1:3,2)*axisLength;
    zaxis = T(1:3,3)*axisLength;

    figure(f);
    h(1) = quiver3(origin(1), origin(2), origin(3),...
                   xaxis(1), xaxis(2), xaxis(3),...
                   'Color', 'r','LineWidth', 2, 'MaxHeadSize', 0.5);
    hold on;
    h(2) = quiver3(origin(1), origin(2), origin(3),...
                   yaxis(1), yaxis(2), yaxis(3),...
                   'Color', 'g','LineWidth', 2, 'MaxHeadSize', 0.5);
    h(3) = quiver3(origin(1), origin(2), origin(3),...
                   zaxis(1), zaxis(2), zaxis(3),...
                   'Color', 'c','LineWidth', 2, 'MaxHeadSize', 0.5);
    if isstring(frameName)||ischar(frameName)
        h(4) = text(origin(1)+0.05*axisLength,...
                    origin(2)+0.05*axisLength,...
                    origin(3)+0.05*axisLength,...
                    strcat('\{', frameName, '\}'),'Interpreter','latex');
        h(5) = text(origin(1)+xaxis(1)*(1+0.05*axisLength),...
                    origin(2)+xaxis(2)*(1+0.05*axisLength),...
                    origin(3)+xaxis(3)*(1+0.05*axisLength),...
                    strcat('$x_{', frameName, '}$'),'Interpreter','latex');
        h(6) = text(origin(1)+yaxis(1)*(1+0.03*axisLength),...
                    origin(2)+yaxis(2)*(1+0.03*axisLength),...
                    origin(3)+yaxis(3)*(1+0.03*axisLength),...
                    strcat('$y_{', frameName, '}$'),'Interpreter','latex');
        h(7) = text(origin(1)+zaxis(1)*(1+0.03*axisLength),...
                    origin(2)+zaxis(2)*(1+0.03*axisLength),...
                    origin(3)+zaxis(3)*(1+0.03*axisLength),...
                    strcat('$z_{', frameName, '}$'),'Interpreter','latex');
    end

end
