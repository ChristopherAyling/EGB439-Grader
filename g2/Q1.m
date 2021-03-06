function myFunction(q0, sensorFunction)
    % q0 is a 3x1 vector giving the initial configuration of the robot in units of metres and degrees
    % sensorFunction is a function that accepts the robot configuration and 
    % returns a vector containing the left and right sensor values
    % eg. sensors = sensorFunction( q )
    % where sensors is a 2x1 vector containing the left and right sensor readings
    
    q = q0';
    qs = [q];
    sensorVals = sensorFunction(q0);
    
    % loop through while still under 0.99
    while (sum(sensorVals < 0.99) == 2)
        q = qupdate(q, 1 - sensorVals)';
        qs = [qs; q];
        sensorVals = sensorFunction(q);
    end
    qs
    plot(qs(:,1), qs(:,2), 'r-*')
end