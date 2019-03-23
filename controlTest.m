goal = [1 1]; % 
% goal = [1 -1]; % 
% goal = [-1 1]; % 
% goal = [-1 -1]; % works
q0 = [0, 0, 0];
q = [0.1 0.1 0];

dt = 0.2;

x = [];
y = [];

xlabel('x'); ylabel('y'); grid on; title('Simulated robot path')
axis([-2 2 -2 2]);
axis square;
hold on

% simulate motion
for i=1:200
    vel = myFunction('control', q, goal);
    q = myFunction('qupdate', q, vel, dt);  % Run reference solution.
    % test if distance to goal is always reducing
    d = norm( [q(1) q(2)] - goal);
    x = [x q(1)]; y = [y q(2)];
    plot(x, y, 'r')
    plot(goal(1), goal(2), 'pk')
    plot(q0(1), q0(2), 'ok')
    pause(0.02);
end

hold off
legend('path', 'goal', 'start')