function [newlist,nodeid] = qinsert(list, nodeid, cost)
    % insert the node id into the list and return an updated list.  The node is inserted
    % at a location such that the cost of the nodes is non-decreasing in magnitude.
    % cost is a vector such that cost(i) is the cost of node i. It is guaranteed that 
    % cost will be a vector with length at least equal to the maximum nodeid.
    % If multiple nodes have the same cost, their relative ordering does not matter.
    % 
    % 
    % Input:
    %   list    (vector)
    %   nodeid  (scalar)
    %   cost    (vector)
    %
    % Output:
    %   newlist (vector)
    for i = 1:length(list)
        if cost(nodeid) < cost(list(i))
            if (i == 1)
                newlist = [nodeid, list];
                return
            else
                newlist = [list(1:i-1), nodeid, list(i:end)];
                return
            end
        end
    end
    newlist = [list, nodeid];
end