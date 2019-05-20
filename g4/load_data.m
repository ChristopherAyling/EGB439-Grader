function load_data()
    global X  
    global odom
    global sensor
    global map
    global num_z
    data    =  load('data.mat');
    X         =  data.xr';
    X         = X(2:end,:);
    odom   = data.odom;
    sensor = data.sensor;
    map     = data.map;
    num_z = length(map);