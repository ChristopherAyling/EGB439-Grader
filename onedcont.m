g = 10;
s = 5;

kv = 1;
ki = 1;

figure;
hold on;

for i = 1:10
    e = sqrt((g-s)^2)-(g-s);
    de = e;Bot
    
    v = (kv*e)+(ki*de);
    clf;
    plot(e,0, 'ro');
end

hold off;
