%mode 0: runs the full simulation.
%mode 1: runs a landmarks initialisation step. 
%mode 2: runs a prediction step.
%mode 3: runs an update step.
function [output] = myFunction(mode)  
    load_data();  
    % Initialization of the robot pose.
    mu =   [0;0;0*pi/180];
    Sigma =diag([0.1 0.1 0.1*pi/180]).^2;
    
    switch mode
        case 0        
            % this simulator runs for 100 steps
            nsteps = 100;
            % the covariance of the process and measurements noise. 
            R = diag([0.5 50*pi/180]).^2;
            Q = diag([0.5, 5*pi/180]).^2;
            % main loop
            for k = 1:nsteps
               %This function returns the distance travelled (d)
               % and the change in angle (dth) between time step k-1 and k.
               [d,dth]  = get_odom(k);    
               % complete the prediction step in the body of the function below
               [mu,Sigma] = predict_step(mu,Sigma,d,dth,R);
                % The sensor measurements to all the landmarks at the 
                % current time step. z is a matrix of shape nx2 
                % where n is the number of landmarks in the map. 
                % The first column in z is the range (m) and
                % the second column is the bearing (rad).
                z          = sense(k);
                
                if k == 1
                    % We use the sensor readings from the first time step 
                    % to initialise all the landmarks in the map. 
                    % this function takes:
                    %   z : The sensor measurements to all the landmarks 
                    %        at the current time step.
                    %   Q: the covariance of the measurements
                    %   mu: contains the predicted pose of the robot
                    % The function returns mu and Sigma after initializing
                    % all the landmarks
                    [mu, Sigma] = initLandmarks(z,Q,mu,Sigma);             
                else                    
                   for i=1:length(z)
                        % i is the id of a landmark
                        % zi  is the range and bearing to landmark i
                        zi     = z(i,:);
                        
                        % this function takes:
                        %   i: id of landmark
                        %   zi: the range and nearing to landmark i
                        %   Q: the covariance of the measurements
                        %   mu,Sigma: the current estimate of the pose of the robot and the map.
                        %   
                        % The function returns mu and Sigma after performing
                        % an update step using the sensor readings of
                        % landmark i                        
                        [mu, Sigma] = update_step(i,zi,Q,mu,Sigma);                        

                   end

                end   
                %******************************* the code below is just for plotting  
                if k == nsteps
                    grid on; axis equal;
                    axis([-2 5 -2 5]);
                    hold on
                    plot_robot(mu)
                    plot_cov(mu,Sigma,3,'b')
                    for i=1:length(z)
                        lidx = 3+2*i;
                        li = mu(lidx-1:lidx);
                        scatter(li(1),li(2),'b+');
                        lSigma = Sigma(lidx-1:lidx,lidx-1:lidx);
                        plot_cov(mu(lidx-1:lidx),lSigma,3,'g');
                    end
                end
            end
            plot_map();             
            output = [mu,Sigma];
        case 1
            Q = diag([0.5, 5*pi/180]).^2;                           
            z          = sense(1);
            [mu, Sigma] = initLandmarks(z,Q,mu,Sigma);
            output = [mu,Sigma];
        case 2            
            R = diag([0.5 50*pi/180]).^2;
            [d,dth]  = get_odom(1);    
            [mu,Sigma] = predict_step(mu,Sigma,d,dth,R);
            output = [mu,Sigma];
        case 3
            Q = diag([0.5,3*pi/180]).^2;
            R = diag([0.5 50*pi/180]).^2;
            [d,dth]  = get_odom(1);    
            [mu,Sigma] = predict_step(mu,Sigma,d,dth,R);
            z          = sense(1);
            [mu, Sigma] = initLandmarks(z,Q,mu,Sigma);            
            
            [d,dth]  = get_odom(2);    
            [mu,Sigma] = predict_step(mu,Sigma,d,dth,R);
            z          = sense(2);
            
            grid on; axis equal;
            axis([-2 5 -2 5]);
            hold on
            plot_robot(mu)
            plot_cov(mu,Sigma,3,'b')
            for i=1:length(z)
                zi     = z(i,:);  
                [mu, Sigma] = update_step(i,zi,Q,mu,Sigma);
                lidx   = 3 + i*2 -1;
                lSigma = Sigma(lidx:lidx+1,lidx:lidx+1);
                plot_cov(mu(lidx:lidx+1),lSigma,3,'g');                
                scatter(mu(lidx),mu(lidx+1),50,'b+');   
            end
            output = [mu,Sigma];            
    end    
end


% --------------- complete the three functions below


% This function takes:
%     mu,Sigma: the current estimate of the pose of the robot and the map.
%     as well as the odometry information (d the distance travelled from time step k-1 and k and dth, the change of heading) 
%     and the matrix R (the covariance of the odometry noise). 
% The function performs a prediction step of the EKF localiser and returns the mean and covariance of the robot pose and the map.   
% note: although the prediction step does not change the estimation 
%      of the landmarks in the map, this function accepts the full state space
%      and only alter the pose of the robot in it.
function [mu,Sigma] =predict_step(mu,Sigma,d,dth,R)
x = mu(1);
y = mu(2);
theta = mu(3);

n = length(Sigma)-3; % number of landmarks * 2

xt = [
    x+(d*cos(theta));
    y+(d*sin(theta));
    wrapToPi(theta+dth);
];

mu = [xt; mu(4:end)];
    
Jx = [
    1 0 -d*sin(xt(3));
    0 1 d*cos(xt(3));
    0 0 1;
];

Jx = [
    Jx zeros(3, n);
    zeros(3, n)' eye(n);
];

Ju = [
    cos(xt(3)) 0;
    sin(xt(3)) 0;
    0 1;
];

Ju = [Ju; zeros(n, 2)];

Sigma = Jx*Sigma*Jx' + Ju*R*Ju';

end
% We use the sensor readings from the first time step 
% to initialise all the landmarks in the map. 
% this function takes:
%   z : The sensor measurements to all the landmarks 
%        at the current time step.
%   Q: the covariance of the measurements
%   mu,Sigma: the current estimate of the robot pose and the map (the map will be empty so the size of mu is 3x1 and Sigma 3x3).
% The function returns mu and Sigma after initialising (if n is the number of landmarks, the function returns mu of size (3+2n)x1 and Sigma of size (3+2n)x(3+2xn))
% all t he landmarks
function [mu, Sigma] = initLandmarks(z,Q,mu,Sigma)
for i=1:length(z)
    x = mu(1);
    y = mu(2);
    theta = mu(3);
    
    r = z(i, 1);
    b = z(i, 2);
    thetaplusb = wrapToPi(theta+b);
    lnew = [
        x+r*cos(thetaplusb);
        y+r*sin(thetaplusb);
    ];
    
    mu = [
        mu;
        lnew;
    ];
    
    zs = zeros(length(Sigma), 2);
    
    L = [
        cos(thetaplusb), -r*sin(thetaplusb);
        sin(thetaplusb), r*cos(thetaplusb);
    ];
    
    Sigma = [
        Sigma zs;
        zs' L*Q*L';
    ];
end
    
    
    
end

% this function takes:
    %  landmarkID: id of a landmark
    %   zi: the range and nearing to this landmark 
    %   Q: the covariance of the measurements
    %   mu,Sigma: the current estimate of the robot pose and the map.
    %   
    % The function returns mu and Sigma after performing
    % an update step using the sensor readings of
    % the landmark   
function [mu, Sigma] = update_step(landmarkID,zi,Q,mu,Sigma)
x = mu(1);
y = mu(2);
theta = mu(3);

xl = mu(3 + landmarkID*2-1);
yl = mu(3 + landmarkID*2);

r = zi(1);
b = zi(2);
h = [
    sqrt((x-xl)^2 + (y-yl)^2);
    wrapToPi(atan2(yl-y, xl-x)-theta);
]';

cr = h(1);
g1 = [
    -(xl-x)/cr, -(yl-y)/cr, 0;
    (yl-y)/(cr^2), -(xl-x)/(cr^2), -1;
];
g2 = [
    (xl-x)/cr, (yl-y)/cr;
    -(yl-y)/(cr^2), (xl-x)/(cr^2);
];
zs = zeros(2, length(Sigma)-3);
G = [
    g1 zs(:, 1:landmarkID*2-2) g2 zs(:, landmarkID*2+1:end);
];

K = Sigma*G'*inv(G*Sigma*G' + Q);
I = eye(length(Sigma));
    
err = zi-h;
err = [err(1) wrapToPi(err(2))]';
mu = mu + K*(err);

Sigma = (I - K*G)*Sigma; 
end


% ----------------------------
% write the extra functions that you need and call them in the three functions above


