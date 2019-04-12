% test qappend function
list = [];
list = Q4('qappend', list, 1)
list = Q4('qappend', list, 2)
list = Q4('qappend', list, 3)

% test qcontains function
Q4('qcontains', list, 2)
Q4('qcontains', list, 7)

% test qpop function
[list,out] = Q4('qpop', list)
[list,out] = Q4('qpop', list)

% test popmin function
list = [5 4 1 3 4];  % create a list of nodes
% create a list of costs
% cost of node 1 is 8
% cost of node 2 is 7
% cost of node 3 is 6 etc.
% the lowest code node is node 3 with a cost of 1
cost = [8 7 4 7 5];
list = [];
list = Q4('qinsert', list, 1, cost)
list = Q4('qinsert', list, 2, cost)
list = Q4('qinsert', list, 3, cost)
list = Q4('qinsert', list, 4, cost)
list = Q4('qinsert', list, 5, cost)

% this should return node 3 and remove it from the list
[list,out] = Q4('qpop', list)

% test neighbours function
D = [0  5  1  0  0
     5  0  2  0  3
     1  2  0  1  0
     0  0  1  0  0
     0  3  0  0  0]

nbours = Q4('neighbours', D, 3)

% create a random distance matrix
A = randi([0 3], [10 10]);
T = triu(A); % get upper triangular part
D = T+T.';   % make it symmetric
D = D - diag(diag(D)); % set diagonal to zero

for n = 1:10
    nbours = Q4('neighbours', D, n);
    assert(size(nbours,1) == 1, 'neighbour list must be a row vector');
    row = D(n,:);
    assert(all(row(nbours) > 0), 'some reported neigbours are not neighbours');
    row(nbours) = [];  % remove the purported non-zero values
    assert(all(row == 0), 'some neigbours have been missed');
end

% qpop
list = randi(100, [1 10]);
[newlist,out] = Q4('qpop', list);
assert(length(newlist) == 9, 'returned list has wrong number of elements');
assert(all(newlist == list(2:end)), 'returned list has incorrect values');
assert(out == list(1), 'popped value was not at the front of the list');

% test 1 element case
list = randi(100);
[newlist,out] = Q4('qpop', list);
assert(length(newlist) == 0, 'returned list has wrong number of elements');
assert(out == list(1), 'popped value was not at the front of the list');

% test empty list case
list = [];
[newlist,out] = Q4('qpop', list);
assert(length(newlist) == 0, 'returned list has wrong number of elements');
assert(isempty(out), 'for empty list the popped value should be []');

% qinsert
for i=1:20
    nodes = randperm(10);  % random node numbers 1-10
    cost = randperm(10);   % random costs
    
    list = [];
    for j=1:10
        list = Q4('qinsert', list, nodes(j), cost);
        cost(list)
    end
    assert(length(list) == 10, 'created list has wrong number of elements');
    assert(all(diff(cost(list)) >= 0), 'returned list has incorrect values');
end