update_rate = 5;
dT = 1/update_rate;
run_time = 30;
nSteps = run_time * update_rate;

A = [
    1 0 dT 0;
    0 1 0 dT;
    0 0 1 0;
    0 0 0 1];

sigmaV = 0.01;
R = [
    0.001 0 0 0;
    0 0.001 0 0;
    0 0 (sigmaV)^2 0;
    0 0 0 (sigmaV)^2;
    ];

figure(1);
hold on
initX = [0 0 0.5 0.5];
x_true = zeros(4, nSteps);
x_true(:,1) = initX;
for i = 2:nSteps
    v = mvnrnd([0 0 0 0], R, 1)';
    x_true(:, i) = A*x_true(:, i-1) + v;
end
scatter(x_true(1,:), x_true(2,:), 'o');