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