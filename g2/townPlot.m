function townPlot(path)
    load queensland_towns
    hold on

    % plot edges
    for nodeid = 1:length(placeNames) % for all points
        % check who the neighbors are
        nbors = neighbours(distanceMatrix, nodeid);
        for nbor = nbors
            % draw a line
            line = [placeCoords(:, nodeid)'; placeCoords(:, nbor)'];
            plot(line(:,1), line(:,2), 'r-')
        end
    end

    % plot nodes
    plot(placeCoords(1,:), placeCoords(2,:), 'bo', 'MarkerSize', 10)
    text(placeCoords(1,:), placeCoords(2,:), placeNames, 'FontSize', 12)
    text(placeCoords(1,:), placeCoords(2,:), num2cell(1:length(placeNames)), 'FontSize', 40)
    
    % plot path
    pathPoints = placeCoords(:, path)';
    plot(pathPoints(:, 1), pathPoints(:, 2), '-mo', 'LineWidth', 2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [.49, 1, .63], 'MarkerSize', 10)
    hold off
end