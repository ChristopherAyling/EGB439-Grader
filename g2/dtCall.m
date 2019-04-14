clf
% 
% map = [
%     0 0 0 0 0
%     0 0 0 0 0
%     0 1 1 1 0
%     0 1 1 1 0
%     0 0 0 1 0
%     0 0 0 1 0
%     0 0 0 0 0
%     ];
start = [65 95];
goal = [3 6];

% test the distance transform
dtransform = Q3('dxform', map, goal);

% test the path generation
path = Q3('path', map, start, goal);

% plot stuff
figure(3)
idisp(dtransform)
hold on
plot(start(1), start(2), 'bo')
plot(goal(1), goal(2), 'rp')
plot(path(:, 1), path(:, 2), 'g*-')

axis square

hold off

assert(size(path,2)==2, 'Path should have 2 columns');
assert(all(path(1,:)== start), 'Path should begin with the start point');
assert(all(path(end,:)== goal), 'Path should end with the goal point');
