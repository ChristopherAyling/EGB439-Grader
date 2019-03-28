wheelVel = myFunction('vw2wheels', [0.2 -0.1])
vw = myFunction('wheels2vw', [30 50])
q = [1 2 0.5]
qd = myFunction('qdot', q, [30 50])
qnew = myFunction('qupdate', q, [30 50], 0.2)
vel = myFunction('control', q, [4 5])

assert(all(size(vw) == [1 2]), 'wheels2vw should return a 1x2 matrix');
assert(all(size(wheelVel) == [1 2]), 'vw2wheels should return a 1x2 matrix');
assert(all(size(qd) == [1 3]), 'qdot should return a 1x3 matrix');
assert(all(size(qnew) == [1 3]), 'qupdate should return a 1x3 matrix');
assert(all(size(vel) == [1 2]), 'control should return a 1x2 matrix');