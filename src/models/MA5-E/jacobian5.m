function J = jacobian5(manipulatorJoints)
    % Geometric jacobian computation    
    [TB0, TB1, TB2, TB3, TB4, TB5] = direct5(manipulatorJoints);

    z = [TB0(1,3) TB0(2,3) TB0(3,3);
         TB1(1,3) TB1(2,3) TB1(3,3);
         TB2(1,3) TB2(2,3) TB2(3,3);
         TB3(1,3) TB3(2,3) TB3(3,3);
         TB4(1,3) TB4(2,3) TB4(3,3);
         TB5(1,3) TB5(2,3) TB5(3,3)]';
     
    p = [TB0(1,4) TB0(2,4) TB0(3,4);
         TB1(1,4) TB1(2,4) TB1(3,4);
         TB2(1,4) TB2(2,4) TB2(3,4);
         TB3(1,4) TB3(2,4) TB3(3,4);
         TB4(1,4) TB4(2,4) TB4(3,4);
         TB5(1,4) TB5(2,4) TB5(3,4)]';

    Jp = zeros(3,5);
    for i = 1:5
        Jp(:,i) = cross(z(:,i),(p(:,6)-p(:,i)));
    end
    
%     Jo = z(:,1:5);

    % Manually generating the orientation jacobian to avoid weird
    % behaviours
    Jo = [0 0 0 0 1;
          0 1 1 1 0;
          1 0 0 0 0];
    
    J = [Jp;
         Jo];
end