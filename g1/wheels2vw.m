function vw = wheels2vw(wheelVel)
    % Inputs:
    % wheelVel is the wheel velocity vector (vleft, vright) each in the range -100 to +100
    % Return:
    % vw is the resulting velocity vector (v, omega) in units of metres/s and radians/s
    W = 0.16; % bot width
    DIAM = 0.065; % wheel diameter
    
    vleft = wheelVel(1);
    vright = wheelVel(2);
    
    vl = pi*DIAM*(vleft/2)*(1/60);
    vr = pi*DIAM*(vright/2)*(1/60);
    
    vd = vr - vl;
    
    thetadot = vd/W;
    
    v = 0.5*(vl+vr);
    
    vw = [v, thetadot];
end