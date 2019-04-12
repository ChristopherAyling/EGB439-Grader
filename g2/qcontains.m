function in = qcontains(list, nodeid)
    % return true if the node id is in the list, otherwise false
    % 
    % Input:
    %   list    (vector)
    %   nodeid  (scalar)
    %
    % Output:
    %   in (logical)
    in = any(list == nodeid);
end