function collide = collision_check(q)
    % q is a 1x3 vector containing the robot configuration with elements x, y and theta (in degrees)
    % collide is true if the robot intersects any of the obstacles, otherwise it is false
    clf;
    hold on;
%     axis equal;
    s1 = [
        2, 5;
        2, 7;
        3, 7;
        3, 5;
    ];

    s2 = [
        5, 0;
        5, 4;
        6, 4;
        6, 0;
    ];

    s3 = [
        5, 6;
        5, 10;
        6, 10;
        6, 6;
    ];

    obstacles = [
        polyshape(s1), polyshape(s2), polyshape(s3),
    ];

    % plot obstacles
    plot(obstacles, 'FaceColor', 'r')
    
    % make bot
    W = 3;
    L = 1.5;
    
    loc = [q(1) q(2)];
    theta = q(3);
    r = [
        cos(theta) -sin(theta);
        sin(theta) cos(theta);
    ];
    
    bot = [
        -L/2 -W/2;
        -L/2 W/2;
        L/2 W/2;
        L/2 -W/2;
    ] + loc;

    botr = [];
    for p = bot'
       botr = [botr; p'*r+loc];
    end

    % rotate bot
    offset = [L/2 W/2];
    refpoint = [bot(1, :) + offset];
    botgon = rotate(polyshape(bot), theta, refpoint);
    
    % plot bot
    plot(botgon, 'FaceColor', 'b')
    
    % check for intersections
    for obstacle = obstacles
        if botgon.intersect(obstacle).area > 0
            collide = true;
            return
        end
    end
    collide = false;
end