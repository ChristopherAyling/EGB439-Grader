SigmaX = 10*eye(4);
muX = [0 0 -10 -5]';

x_est = [];
x_est = [x_est, muX];

for i = 1:nSteps
   z = sensor(:,i);
   % predict step
   muX = A*muX;
   SigmaX = A*SigmaX*A' + R;
   
   % correction step
   K1 = SigmaX * H';
   K2 = H * SigmaX * H' + Q;
   K = K1/K2;
   r = z - H * muX;
   muX = muX + k * r;
   SigmaX = (eye(4) - K*H) * SigmaX;
   x_est = [x_est, muX];
   scatter(muX(1), muX(2), 'b*');
   plot_cov(muX, SigmaX, 3)
end
plot(x_est(1, :), x_est(2, :))