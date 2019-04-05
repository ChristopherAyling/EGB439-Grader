map = [
0 0 0 0 0
0 0 0 0 0
0 1 1 1 0
0 1 1 1 0
0 0 0 1 0
0 0 0 1 0
0 0 0 0 0
];
goal = [4 2];

% Run learner solution.
% check dimensions
% check first point is start, last point is goal
dx = Q3('dxform', map, goal);

assert(all(size(map)==size(dx)), 'Distance transform result not same size as input occupancy grid');
assert(any(isnan(dx(:))), 'Use NaN to represent occupied cells in the distance transform');
assert(~any(isinf(dx(:))), 'Distance transform result contain infinite values');