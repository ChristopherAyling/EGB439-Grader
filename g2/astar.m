function [path,frontierList,exploredList] = astar(distanceMatrix, placeCoords, placeNames, startNode, goalNode)
    
    parentNode = [];  % parentNode(i) is the number of the node which is the parent of node i
    
    frontierList = startNode;  % put the starting node in the frontier
    exploredList = [];  % nothing has been explored yet
    g(startNode) = 0; % distance travelled to reach each node
    h(startNode) = pointDist(placeCoords(:, startNode)', placeCoords(:, goalNode)');
    
    % do the breadth first search
    while length(frontierList) > 0  % while the frontier is not empty
        
        % get next node from frontier list, this is the node we are expanding
        [frontierList, expandingNode] = qpop(frontierList);
        fprintf('expanding node %d (%s)\n', expandingNode, placeNames{expandingNode});
        
        % check if we are there yet?
        if expandingNode == goalNode
            break;  % yes
        end
        
        % visit all the neighbours of the node being expanded
        for n = neighbours(distanceMatrix, expandingNode)  % for each neighbour
            if ~qcontains(frontierList, n) && ~qcontains(exploredList, n)
                % this neighbour is not on the frontier and is not explored
                % cost to come
                g(n) = g(expandingNode) + distanceMatrix(expandingNode, n);
                % cost to go
                h(n) = pointDist(placeCoords(:, n)', placeCoords(:, goalNode)')/100/100/10;
                
                fprintf('  adding node %d (%s) to frontier, %.0f from start\n', n, placeNames{n}, g(n));
                frontierList = qinsert(frontierList, n, g+h);
                parentNode(n) = expandingNode;  % record how we got to this node
            end
        end
        
        % we are done with this node now, add it to the explored list
        exploredList = qappend(exploredList, expandingNode);
        fprintf('  adding node %d %s to explored list\n', expandingNode, placeNames{expandingNode});
    end
    
    % Now we need to reconstruct the path.  For each node we record it's parent
    % node, the node we arrived from.  Starting at the goal node, we find its
    % parent, then the parent of that node, until we get to the start node
    
    path = [];
    n = goalNode;  % set current node to the start

    while true
        path = [n path];    % prepend it to the path
        if n == startNode   % quit now if we have reached the starting node
            return
        end
        n = parentNode(n);  % find the parent of this node
    end
end


function newlist = qappend(list, nodeid)
    % add the node id to the end of the list and return an updated list 
    %
    % Input:
    %   nodeid  (scalar)
    %   list    (vector)
    %
    % Output:
    %    newlist (vector)
    newlist = [list, nodeid]
end

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

function [newlist,nodeid] = qpop(list)
    % get a node id from the front of list and return an updated list. If the list is
    % empty return newlist as and nodeid as the empty value [].
    % 
    % Input:
    %   list    (vector)
    %
    % Output:
    %   newlist (vector)
    %   nodeid  (scalar)
    if isempty(list)
       newlist = [];
       nodeid = [];
    else
        newlist = list(2:end);
        nodeid = list(1);
    end
end



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

function d = pointDist(p1, p2)
    d = sqrt(sum((p1-p2) .^2));
end