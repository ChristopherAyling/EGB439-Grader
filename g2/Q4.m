% this wrapper function allows the assessment code to access your functions
function [out1,out2] = myFunction(op, varargin)
    switch op
        case 'qappend'
            out1 = qappend(varargin{:});
        case 'qcontains'
            out1 = qcontains(varargin{:});
        case 'qpop'
            [out1,out2] = qpop(varargin{:});
        case 'qinsert'
            out1 = qinsert(varargin{:});
        case 'neighbours'
            out1 = neighbours(varargin{:});            
    end
end

% YOUR CODE GOES BELOW

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