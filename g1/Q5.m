clf;

LENGTH = 4;
WIDTH = 1.7;
WB = 3;
BAXOFFSET = 0.5;

q = [0, 0, 0];
velocityInitial = -0.7;
thetaInitial = 15;

dt = 0.05;

% plot setup
axis([-5 5 -5 5])
axis square;
hold on;

% plot origin
plot(q(1), q(2), 'bp');

% plot Car
p1 = [q(1) q(2)] + [-BAXOFFSET, -WIDTH/2];
plot(p1(1), p1(2), 'bo') % back right corner

p2 = [q(1) q(2)] + [-BAXOFFSET, +WIDTH/2];
plot(p2(1), p2(2), 'bo') % back left corner

p3 = [q(1) q(2)] + [-BAXOFFSET + LENGTH, -WIDTH/2];
plot(p3(1), p3(2), 'bo') % front right corner

p4 = [q(1) q(2)] + [-BAXOFFSET + LENGTH, +WIDTH/2];
plot(p4(1), p4(2), 'bo') % front left corner

points = [p1; p2; p4; p3; p1];

plot(points(:,1), points(:,2), 'b-', 'Tag', 'Car')

% calculate path

qs = [
    q;
    q-1;
    q-2;
    q-3;
    q-4;
    q-5;
];

plot(qs(:, 1), qs(:, 2), 'g.')

% plot back left corner path
blc = qs(:, 1:2) + [-BAXOFFSET, +WIDTH/2];
plot(blc(:,1), blc(:,2), 'r.-', 'Tag', 'Left');

% plot back right corner path
brc = qs(:, 1:2) + [-BAXOFFSET, -WIDTH/2];
plot(brc(:,1), brc(:,2), 'r.-', 'Tag', 'Right');

% plot back edge of car at time 1 second
backEdge = [blc(2, :); brc(2, :)];
plot(backEdge(:,1), backEdge(:,2), 'k.-', 'Tag', '1Second');

hold off;