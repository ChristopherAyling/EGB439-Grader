function qd = qdotCar(q, vw)
    v = vw(1);
    w = vw(2);
    theta = q(3);
    
    xdot = v*cos(theta);
    ydot = v*sin(theta);
    w = (0.7/4) * tan(theta)
    qd = [xdot, ydot, w];
end

