function q = inverso3(p, modo)
    % Modelo cinematico inverso
    a1 = 0.20;
    a2 = 0.15;
    a3 = 0.10;

    x = p(1);
    y = p(2);
    phi = p(3);

    d = sqrt(x^2+y^2);
    if d > (a1 + a2 + a3)
        fprintf('Localizacion fuera de alcance');
        return
    end

    xm = x - a3*cos(phi);
    ym = y - a3*sin(phi);

    dm = sqrt(xm^2+ym^2);
    if dm > (a1 + a2)
        fprintf('Orientacion fuera de alcance');
        return
    end

    q2 = acos((xm^2+ym^2-a1^2-a2^2)/(2*a1*a2));
    beta = atan2(ym,xm);
    alpha = acos((a2^2-a1^2-xm^2-ym^2)/(-2*a1*dm));

    % Generacion de las soluciones articulares
    % en funcion del parametro 'modo'
    if modo > 0
        % Solucion positiva.
        q = [beta - alpha, q2, phi - (beta-alpha) - q2];
    elseif modo < 0
        % Solucion negativa.
        q = [beta+alpha,-q2, phi - (beta+alpha) + q2];
    end
end