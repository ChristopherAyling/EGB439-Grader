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
function vel = control(q, point)
    % Inputs:
    % q is the initial configuration vector (x, y, theta) in units of metres and radians
    % point is the vector (x, y) specifying the goal point of the robot
    vmax = 100;
    
    x = q(1);
    y = q(2);
    theta = q(3);
    gx = point(1);
    gy = point(2);
    
    xdiff = gx-x;
    ydiff = gy-y;
    
    KV = 10;
    KH = 100;
    
    derror = sqrt(xdiff^2+ydiff^2);
    if derror <= 0.01
       vel = vw2wheels([0, 0]);
       return
    end
    agoal = atan2(ydiff, xdiff);
    aerror = wrapToPi(agoal - theta);
    
    v = KV * derror;
    w = KH * aerror;
    
    vel = vw2wheels([v, w]*(1/60));
end
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
function qnew = qupdate(q, vel, dt)
    % Inputs:
    % q is the configuration vector (x, y, theta) in units of metres and radians
    % vel is the velocity vector (vleft, vright) each in the range -100 to +100
    % dt is the length of the integration timestep in units of seconds
    % Return:
    % qnew is the new configuration vector vector (x, y, theta) in units of metres and radians at the
    % end of the time interval.
    qd = qdot(q, vel);
    qnew = q+(qd.*dt);
end
