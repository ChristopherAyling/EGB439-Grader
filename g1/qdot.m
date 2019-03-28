function qd = qdot(q, vel)
    % Inputs:
    % q is the configuration vector (x, y, theta) in units of metres and radians
    % vel is the velocity vector (vleft, vright) each in the range -100 to +100
    % Return:
    % qd is the vector (xdot, ydot, thetadot) in units of metres/s and radians/s
    vw = wheels2vw(vel);
    v = vw(1);
    w = vw(2);
    theta = q(3);
    
    xdot = v*cos(theta);
    ydot = v*sin(theta);
    
    qd = [xdot, ydot, w];
end