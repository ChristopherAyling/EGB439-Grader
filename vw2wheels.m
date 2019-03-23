function wheelVel = vw2wheels(vw)
    % Inputs:
    % vw is the velocity vector (v, omega) in units of metres/s and radians/s
    % Return:
    % wheelVel is the wheel velocity vector (vleft, vright) each in the range -100 to +100 to achieve
    % this velocity
    v = vw(1);
    w = vw(2);
    W = 0.16;
    DIAM = 0.065;
    
    X = [2*v; w*W];
    A = [1, 1; -1, 1];
    B = A \ X;
    
    vs = B'*2 / (pi*DIAM*(1/60));
    
    if vs(1) < -100
       vs(1) = -100;
    elseif vs(1) > 100
        vs(1) = 100;
    end
    
    if vs(2) < -100
       vs(2) = -100;
    elseif vs(2) > 100
        vs(2) = 100;
    end
    
    wheelVel = vs;
end