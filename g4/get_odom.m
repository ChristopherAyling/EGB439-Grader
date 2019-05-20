function [delta_d,delta_th] = get_odom(k)
    % returnS odometry at time step k
    global odom   
    delta_d   = odom(k,1);
    delta_th  = odom(k,2);