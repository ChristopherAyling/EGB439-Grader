function out = myFunction(op, varargin)
    switch op
        case 'dxform'
            out = distanceTransform(varargin{:});
        case 'path'
            out = findPath(varargin{:});
    end
end

%----- YOUR CODE GOES BELOW HERE -----

function dtransform = distanceTransform(map, goal)
    % Input:
    %   map is an arbitrary size real matrix representing an occupancy grid. The elements are equal to 0 (free space) or 1 (occupied)
    %   goal is a 1x2 matrix containing the end coordinate [x y] for the path
    % Return:
    %   dtransform is a matrix, the same size as map, whose elements reflect the distance from that cell to the goal
    
    % compute the distance transform of map
    
    % pad with NaNs
    map = padarray(map, [1, 1], NaN, 'both');
    
    % take padding into account
    goal = goal + 1;
    
    % make 1s into NaNs
    map(map==1) = NaN;
    
    % make 0s into infs
    map(map==0) = inf;
    
    % set goal to 0
    map(goal(2), goal(1)) = 0;
    
    
    sz = size(map);
    xlen = sz(2);
    ylen = sz(1);

    is = 1:xlen;
    js = 1:ylen;
    
    % make kernel (manhattan)
    kernel = [
        inf 1 inf;
        1 0 1;
        inf 1 inf;
    ];
    
    % do distance transform
    STEPS = 300;
    for step = 1:STEPS
        for i=is
            for j=js
                if isnan(map(j, i))
                   continue 
                end
                W = window(map, j, i);
                cost = calcCost(W, kernel);
                map(j, i) = cost;
            end
        end
    end
    
    % remove padding
    map = map(2:end-1, 2:end-1);
    
    dtransform = map;
end

function path = findPath(map, start, goal)
    % Input:
    %   map is an arbitrary size real matrix representing an occupancy grid. The elements are equal to 0 (free space) or 1 (occupied)
    %   start is a 1x2 matrix containing the start coordinate [x y] for the path
    %   goal is a 1x2 matrix containing the end coordinate [x y] for the path
    % Return:
    %   path is an Nx2 matrix containing the ordered path from start to goal, inclusive of end points.  N depends on the map.
    
    dtrans = distanceTransform(map, goal);   % use your own distance transform function
    
    sx = start(1);
    sy = start(2);
    gx = goal(1);
    gy = goal(2);
    
    % from the starting point, move to the adjacent cell that's closest to
    % the goal
    path = [sx, sy];
    timeout = 0;
    while true
        timeout = timeout + 1;
        current = path(end, :);
        cx = current(1);
        cy = current(2);
        % get next step direction
        W = window(padarray(dtrans, [1, 1], NaN, 'both'), cy+1, cx+1);
        
        % visualise
%         monitor(dtrans, W, map, cx, cy)
       
        nextStep = minval(W);

        % get next step coord
        next = [cx, cy] + nextStep;

        % add to path
        path = [path; next];

        % check if done
        if path(end, :) == goal
          break 
        end

        % stop infinite loop
        if timeout > 1000
           path
           qwerty = 'timed out!'
           break
        end
    end
end

function cost = calcCost(W, kernel)
    cost = min(min(W + kernel));
end

function M = window(A, x, y) 
% Input:
%  A an arbitrary sized real matrix, at least 3x3.
%  x the x-coordinate (horizontal index) of the centre of the window.
%  y the y-coordinate (vertical index) of the centre of the window.
% Return:
%  M as 3x3 matrix which are the elements of M centered on the element (x,y).
%
% Should the window extend beyond the edges of the matrix the function must
% return an empty matrix [].
    sz = size(A);
    xlen = sz(2);
    ylen = sz(1);
    
    wx = x;
    wy = y;
    
    xs = wx-1; 
    xe = wx+1;
    ys = wy-1;
    ye = wy+1;
    
    if ys < 1
        ys = 1;
    end
    if xs < 1
        xs = 1;
    end
%     if ye == 129
%         ye = 128;
%     end
%     if xe == 129
%         xe = 128;
%     end
    
    xmin = 2;
    ymin = 2;
    xmax = xlen-1;
    ymax = ylen-1;
    
    base = [
      inf inf inf;
      inf inf inf;
      inf inf inf;
    ];
    
    isOutOfBounds = wx < xmin || wy < ymin || wx > xmax || wy > ymax;
    isOutOfBounds = false;
    if (isOutOfBounds)
        M = [];
    else
        M = A(xs:xe, ys:ye);
        msz = size(M);
        base(1:msz(1), 1:msz(2)) = M;
        M = base;
    end
end

function next = minval(M)
% Input:
%  M is a real 3x3 matrix
% Return:
%  next is a 1x2 matrix with elements [x, y] which are the horizontal and vertical coordinates relative to the centre
%       element, of the smallest element in the matrix.
    lookup = [
      -1, -1;
      -1, 0;
      -1, 1;
      0, -1;
      0, 0;
      0, 1;
      1, -1;
      1, 0;
      1, 1;
    ];
    mask = [
        inf 0 inf;
        0 0 0;
        inf 0 inf;
    ];
    M = mask + M;
    smallest = min(min(M));
    idx = find(M==smallest);
    loc = lookup(idx, :);
    if size(loc, 1) > 1 % if multiple closest
        next = loc(1, :); % get the first
    else
        next = loc;
    end
end

function monitor(dt, w, map, x, y)
    figure(1)
    subplot(3,1,1), image(map,'CDataMapping','scaled')
    hold on
    plot(x, y, 'rp')
    colorbar
    axis square
    subplot(3,1,2), image(dt,'CDataMapping','scaled')
    colorbar
    axis square
    subplot(3,1,3), image(w,'CDataMapping','scaled')
    colorbar
    axis square
    hold off
end