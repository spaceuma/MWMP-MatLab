function J = jacobian3(manipulatorJoints)
    % Geometric jacobian computation    
    [TB0, TB1, TB2, TB3] = direct3(manipulatorJoints);

    z = [TB0(1,3) TB0(2,3) TB0(3,3);
         TB1(1,3) TB1(2,3) TB1(3,3);
         TB2(1,3) TB2(2,3) TB2(3,3);
         TB3(1,3) TB3(2,3) TB3(3,3)]';
     
    p = [TB0(1,4) TB0(2,4) TB0(3,4);
         TB1(1,4) TB1(2,4) TB1(3,4);
         TB2(1,4) TB2(2,4) TB2(3,4);
         TB3(1,4) TB3(2,4) TB3(3,4)]';

    Jp = zeros(3,3);
    for i = 1:3
        Jp(:,i) = cross(z(:,i),(p(:,4)-p(:,i)));
    end
    
    Jo = z(:,1:3);
    
    J = [Jp;
         Jo];
end