% this wrapper function allows the assessment code to access your functions
function out = myFunction(op, varargin)
    switch op
        case 'window'
            out = window(varargin{:});
        case 'minwin'
            out = minwin(varargin{:});
        case 'minval'
            out = minval(varargin{:});
    end
end

%----- YOUR CODE GOES BELOW HERE -----

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
    
    xmin = 2;
    ymin = 2;
    xmax = xlen-1;
    ymax = ylen-1;
    
    isOutOfBounds = wx < xmin || wy < ymin || wx > xmax || wy > ymax;
    if (isOutOfBounds)
        M = [];
    else
        M = A(ys:ye,xs:xe);
    end
end

function B = minwin(A) 
% Input:
%  A returns a matrix the same size as A where each element of B is the minimum 
% of a 3x3 window of A centered at the corresponding element.  Elements of B 
% that correspond to a window which "falls off the edge" of the matrix A should be set to a value of NaN.
    new = zeros(size(A));
    new(1,:) = NaN;
    new(end,:) = NaN;
    new(:,1) = NaN;
    new(:,end) = NaN;
    
    sz = size(A);
    xlen = sz(2);
    ylen = sz(1);
    xs = 2:xlen-1;
    ys = 2:ylen-1;
    for i=xs
        for j=ys
            new(j, i) = min(min(window(A, i, j)));
        end
    end
    
%     inner = A(2:end-1, 2:end-1);
    
%     new(2:end-1, 2:end-1) = inner;

    B = new;
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
    smallest = min(min(M));
    idx = find(M==smallest);
    loc = lookup(idx, :);
    next = loc;
end
