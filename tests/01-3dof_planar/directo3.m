function [T01, T02, T03] = directo3(q)
    % Modelo cinematico directo
    a1 = 0.20;
    a2 = 0.15;
    a3 = 0.10;

    T01=[cos(q(1)) -sin(q(1)) 0 cos(q(1))*a1 ;
         sin(q(1)) cos(q(1)) 0 sin(q(1))*a1 ;
         0 0 1 0 ;
         0 0 0 1];

    T12=[cos(q(2)) -sin(q(2)) 0 cos(q(2))*a2 ;
         sin(q(2)) cos(q(2)) 0 sin(q(2))*a2 ;
         0 0 1 0 ;
         0 0 0 1];

    T23=[cos(q(3)) -sin(q(3)) 0 cos(q(3))*a3 ;
         sin(q(3)) cos(q(3)) 0 sin(q(3))*a3 ;
         0 0 1 0 ;
         0 0 0 1];

    T02 = T01*T12;
    T03 = T02*T23;
end