function x = ask_the_oracle(k)
    % x is the pose of the robot at time step k
    global X
    x = X(k,:)';