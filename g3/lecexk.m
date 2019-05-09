% what we think the initial state and covariance is
SigmaX = 0.01 * eye(4);
muX = [10 5 2 2]';

R = 1; % I think this is randomness

% track our gueses
x_est = [];
x_est = [x_est, muX];

% begin simulation
nSteps = 100;
for i = 1:nSteps
   % make a prediction
   muX = A*muX; % not taking control into account
   SigmaX = A*SigmaX*A'R;
   
   % read from sensor
   z = sensor(:, i);
   
   % update our prediction with sensor readings
   K1 = SigmaX*H';
   K2 = H * SigmaX * H' + Q;
   K = K1/K2; % kalman gain
   
   muX = muX + k*(z-H*muX);
   SigmaX = (eye(4) - K*H) * SigmaX;
   x_est = [x_est, muX];
end