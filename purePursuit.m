function vw = purePursuit(goal, q, d, dt, first)
% Inputs:
%  goal is a 2x1 vector giving the current point along the path
%  q is a 1x3 vector giving the current configuration of the robot
%  d is the pure pursuit following distance
%  dt is the time between calls to this function
%  first is true (1) on the first call in a simulation, otherwise false (0)
% Return:
%  vw is a 1x2 vector containing the request velocity and turn rate of the robot [v, omega]
    persistent ei
    if first
        ei = 0;
    end
    
    KVp = 1;
    KVi = 0.1;
    KHp = 8;
    
    % decontsruct args
    x = q(1);
    y = q(2);
    theta = q(3);
    
    gx = goal(1);
    gy = goal(2);
    
    % calculate goals and errors
    xdiff = gx-x;
    ydiff = gy-y;
    agoal = atan2(ydiff, xdiff);
    
    derror = sqrt(xdiff^2 + ydiff^2) - d;
    aerror = wrapToPi(agoal - theta);
    
    % calculate velocity
    ei = ei + dt*derror;
    if ei > 1
        ei = 0.6;
    end
    v = KVp * derror + KVi*ei;
    
    % calculate angular velocity
    w = KHp * aerror + KVi*ei;
    
    % deal with janky persistant variables
    if isempty(v)
    v = 0;
    else
        v = v(1);
    end
    
    vmin = -0.5;
    vmax = 0.5;
    wmin = -1;
    wmax = 1;
    
    if v > vmax
        v = vmax;
    elseif v < vmin
        v = vmin;
    end
    
    if w > wmax
        w = wmax;
    elseif w < wmin
        w = wmin;
    end
    
    vw = [v w];
end