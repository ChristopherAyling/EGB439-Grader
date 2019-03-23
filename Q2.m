A = 0.5/sqrt(2);

STEPS = 100;
ts = linspace(0, 2*pi, STEPS);

xs = zeros(1, STEPS);
ys = zeros(1, STEPS);

i = 1;
for t = ts
    xs(i) = (A*sqrt(2)*cos(t))/((sin(t)^2)+1);
    ys(i) = (A*(sqrt(2))*cos(t)*sin(t))/(((sin(t)^2)+1));
    i = i+1;
end

plot(xs, ys);