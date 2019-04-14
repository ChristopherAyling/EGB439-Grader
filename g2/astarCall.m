load queensland_towns

startid = find([placeNames{:}] == "Camooweal");
goalid = find([placeNames{:}] == "Brisbane");

[p,f,e]=graph_planner(distanceMatrix, placeCoords, placeNames, startid, goalid)

figure(1)
clf
title('BFS')
townPlot(p)

[p,f,e] = astar(distanceMatrix, placeCoords, placeNames, startid, goalid)

figure(2)
clf
title('A*')
townPlot(p)