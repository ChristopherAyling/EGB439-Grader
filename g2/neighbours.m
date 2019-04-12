function nodeid = neighbours(distanceMatrix, nodeid)
    % Return a list of node id's for the neighbours of the given node id
    %
    % Input:
    %   distanceMatrix  (square symmetric matrix)
    %   nodeid          (scalar)
    %   
    % Output:
    %   nodeid   (scalar)
    row = distanceMatrix(nodeid, :);
    nodeid = find(row > 0);
end