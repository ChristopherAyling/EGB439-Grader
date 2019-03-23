L = 0.5;
W = 0.3;
AXLE = 0.1;

TARG = [3.1, 3.2];

Q = [2.1, 1.3, 40];
SQ = [L-AXLE -(W/2) -5];

TR = [
    cosd(Q(3)) -sind(Q(3)) Q(1);
    sind(Q(3)) cosd(Q(3)) Q(2);
    0 0 1;
]

TS = TR * [
    cosd(SQ(3)) -sind(SQ(3)) SQ(1);
    sind(SQ(3)) cosd(SQ(3)) SQ(2);
    0 0 1;
]

targh = [TARG(1); TARG(2); 1];
targhom = inv(TS)*targh;
[t, r] = cart2pol(targhom(1), targhom(2));

PP = [
    r; t*(180/pi)
]