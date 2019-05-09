thingo = @(n) ((-1)^n)*(1/(n^2))+69;

things = []
for n = 1:100
   things = [things, thingo(n)]
end

plot(things)