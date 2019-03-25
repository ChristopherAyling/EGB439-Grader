load('purepursuit.mat');
% robot_traj = [robot_traj; robot_traj]

goalCounter = 1;
goal = robot_traj(goalCounter, :);

q0 = [-0, -0, 0];
q = [q0(1) q0(2) 0];

x = [q0(1)];
y = [q0(2)];

dt = 0.25;
d = 0.1;
first = true;

STEPS = 500;
for i=1:STEPS
    % execute controller
    vw = purePursuit(goal, q, d, dt, first);
    vel = vw2wheels(vw);
    first = false;
    
    % simulate motion
    q = qupdate(q, vel, dt);
    
    % track path
    x = [x q(1)]; y = [y q(2)];
    
    % Plot everything 
    path = [x;y]';
    qplot(q, goal, q0, path, robot_traj);
    
    % check if close enough
    xerror = q(1) - goal(1);
    yerror = q(2) - goal(2);
    dist = sqrt(xerror^2 + yerror^2);
    if dist <= d
       goalCounter = goalCounter + 1;
       goal = robot_traj(goalCounter, :);
%        first = true;
    end
    pause(0.0001);
end