%this function will test your implementation of the ekf-localiser 
%complete the tow functions predict_step and update_step. Also, write any extra functions you need. For example, you can write functions to calculate the jacobians.
function [output] = myFunction(mode)    
    load_data();  
    map = get_map();
    switch mode
        % case 0 test the full simulation whereas case 1 and 2 test only one step. 
        case 0        
            % this simulator run for 50 steps.
            nsteps = 50;        
            scatter(map(:,1),map(:,2),200,'k*');
            hold on
            % pose of the robo at time step 0
            x = [0 0 0]'; 
            % The initial covariance matrix on the robot pose
            S = diag([1 1 5*pi/180]).^2;
            % The covariance matrices of the process and the measurement noise 
            R = diag([.01 3*pi/180]).^2;
            Q = diag([.1 3*pi/180]).^2;
            % run for 50 steps
            for k = 1:nsteps
                [d,dth]  = get_odom(k);
                    
                [x,S] = predict_step(x,S,d,dth,R);
                % The sensor measurements to all the landmarks at the 
                % current time step. z is a matrix of shape nx2 
                % where n is the number of landmarks in the map. 
                % The first column in z is the range (m) and
                % the second column is the bearing (rad).    
                z = sense(k);
        
                [x,S] = update_step(map,z,x,S,Q);    
                
                plot_cov(x,S,3);     
                plot_robot(x,15)
            end
            output = [x,S];
        case 1
            x = [0 0 0]'; 
            S = diag([1 1 5*pi/180]).^2;
            R = diag([.01 3*pi/180]).^2; 
            [d,dth]  = get_odom(1);
            [x,S] = predict_step(x,S,d,dth,R);
            output = [x,S];        
        case 2
            Q = diag([.1 3*pi/180]).^2;
            x = [0 0 0]'; 
            S = diag([1 1 5*pi/180]).^2;
            z = sense(1);
            [x,S] = update_step(map,z,x,S,Q);
            output = [x,S]
    end    
end
      
% This function takes:
%     the mean, and a covariance matrix of the robot pose 
%     as well as the odometry information (d the distance travelled from time step k-1 and k and dth, the change of heading) 
%     and the matrix R (the covariance of the odometry noise). 
% The function performs a prediction step of the EKF localiser and returns the mean and covariance of the robot.      
function [xt,S] = predict_step(xt,S,d,dth,R)
x = xt(1);
y = xt(2);
theta = xt(3);

xt = [
    x+(d*cos(theta));
    y+(d*sin(theta));
    wrapToPi(theta+dth);
];
    
Jx = [
    1 0 -d*sin(theta);
    0 1 d*cos(theta);
    0 0 1;
];

Ju = [
    cos(theta) 0;
    sin(theta) 0;
    0 1;
];

S = Jx*S*Jx' + Ju*R*Ju'; 
    
end

   
% This function takes:
%     The location of all the landmarks (map)
%     The sensor readings of the range and bearing to all the landmarks in the map at the current time step.
%     The mean, and a covariance matrix of the robot pose
%     and the matrix Q (the covariance of the sensor noise). 
%
% The function performs an update step of the EKF localiser and returns the mean and covariance of the robot. 
function [x,S] = update_step(map,z,x,S,Q)

for i=1:size(z,1)
    xr = x(1);
    yr = x(2);
    theta = x(3);
    lm = map(i,:);
    
    r = z(i, 1);
    b = z(i, 2);
    
    xl = lm(1);
    yl = lm(2);
    
    G = [
        -(xl-xr)/r, -(yl-yr)/r, 0;
        (yl-yr)/(r*r), -(xl-xr)/(r*r), -1;
    ];
    
    h = [
        sqrt((xl-xr)^2+(yl-yr)^2)
        wrapToPi(atan2(yl-yr, xl-xr)-theta)
    ];
    
    K = S*G'*(G*S*G' + Q)^-1;
    
    err = z(i, :)-h';
    err = [err(1); wrapToPi(err(2))];
    x = x + K*(err);
    x = [x(1), x(2), wrapToPi(x(3))]';
    S = (eye(length(K)) - K*G)*S;
end

end    

% ----------------------------
% write the extra functions that you need and call them in the above two functions





function plot_cov(x,P,nSigma)
    P = P(1:2,1:2); 
    x = x(1:2);
    if(~any(diag(P)==0))
        [V,D] = eig(P);
        y = nSigma*[cos(0:0.1:2*pi);sin(0:0.1:2*pi)];
        el = V*sqrtm(D)*y;
        el = [el el(:,1)]+repmat(x,1,size(el,2)+1);
        line(el(1,:),el(2,:));
    end;
end
