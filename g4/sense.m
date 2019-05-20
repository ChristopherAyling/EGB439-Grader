function z = sense(k)
    % get the sensor data at time step k
    global sensor
    global num_z 
    idx = 1 + (k-1) * num_z;
    z = sensor(idx:idx+num_z-1,:);