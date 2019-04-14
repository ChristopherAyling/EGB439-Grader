p1 = placeCoords(:, 2)';
p2 = placeCoords(:, 3)';

figure(3)
hold on
plot(p1(1), p1(2), 'r*')
plot(p2(1), p2(2), 'bo')
hold off

actualCost = distanceMatrix(2, 3)

dist = ceil(pointDist(p1, p2))