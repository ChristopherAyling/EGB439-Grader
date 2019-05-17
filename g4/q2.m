function [output] = myFunction(mode)    
    load_data();  
    
    switch mode
        % case 0 test the full simulation whereas case 1 and 2 test only one step. 
        case 0        
            % this simulator runs for 50 steps
            nsteps = 50;
            % the map is not given, plotting is just to tell us how
            % accurate is our solution 
            plot_map();           
            % the covariance of the measurements noise. 
            Q = diag([0.5,3*pi/180]).^2;
            % 
            mu = [];
            Sigma = [];            
            for k = 1:nsteps
                % the true pose is given
                xr          = ask_the_oracle(k);    
                plot_robot(xr)    
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
                    %   xr: the true pose of the robot [x;y;theta].
                    % The function returns mu and Sigma after initializing
                    % all t he landmarks
                    [mu, Sigma] = initLandmarks(z,Q,xr);
                else                    
                    for i=1:length(z)
                        % i is the id of a landmark
                        % zi  is the range and bearing to landmark i
                        zi     = z(i,:);
                        
                        % this function takes:
                        %   i: id of landmark
                        %   zi: the range and nearing to landmark i
                        %   Q: the covariance of the measurements
                        %   mu,Sigma: the current estimate of the map.
                        %   xr: the true pose of the robot [x;y;theta].
                        % The function returns mu and Sigma after performing
                        % an update step using the sensor readings of
                        % landmark i                        
                        [mu, Sigma] = update_step(i,zi,Q,mu,Sigma,xr);                        
                        
                        if k == nsteps
                            lidx   = i*2 -1;
                            lSigma = Sigma(lidx:lidx+1,lidx:lidx+1);
                            plot_cov(mu(lidx:lidx+1),lSigma,3);                
                            scatter(mu(2*i-1),mu(2*i),200,'k+');                                        
                        end
                    end
                end                
            end
            output = [mu,Sigma];
        case 1
            Q = diag([0.5,3*pi/180]).^2;
            xr          = ask_the_oracle(1);                
            z          = sense(1);
            [mu, Sigma] = initLandmarks(z,Q,xr);
            output = [mu,Sigma];
        case 2
            Q = diag([0.5,3*pi/180]).^2;
            xr          = ask_the_oracle(1);                
            z          = sense(1);
            [mu, Sigma] = initLandmarks(z,Q,xr);
            
            xr          = ask_the_oracle(1);
            z          = sense(2);  
            plot_map();
            plot_robot(xr)
            for i=1:length(z)
                zi     = z(i,:);
                [mu, Sigma] = update_step(i,zi,Q,mu,Sigma,xr);
                lidx   = i*2 -1;
                lSigma = Sigma(lidx:lidx+1,lidx:lidx+1);
                plot_cov(mu(lidx:lidx+1),lSigma,3);                
                scatter(mu(2*i-1),mu(2*i),200,'k+');   
            end
            output = [mu,Sigma];            
    end    
end
% ---------------
function [mu, Sigma] = initLandmarks(z,Q,xr)
x = xr(1);
y = xr(2);
theta = xr(3);

mu = [];
Sigma = [];

for i = 1:length(z)
    r = z(i, 1);
    b = z(i, 2);
    
    % init mu
    lnew = [
       x+r*cos(theta+b)
       y+r*sin(theta+b)
    ];

    mu = [
       mu;
       lnew;
    ];

    % init Sigma
    nrows = size(Sigma, 1);
    ncols = 2;
    zs = zeros(nrows, ncols);
    L = [
        cos(theta+b) -r*sin(theta+b);
        sin(theta+b) r*cos(theta+b);
    ];
    snew = L*Q*L';
    Sigma = [
        Sigma zs;
        zs' snew;
    ];
end
   
    
    
end 
function [mu, Sigma] = update_step(landmarkID,zi,Q,mu,Sigma,xr)
x = xr(1);
y = xr(2);
theta = xr(3);

r = zi(1);
b = zi(2);

xl = mu(landmarkID*2-1);
yl = mu(landmarkID*2);

g = [
    -(xl-x)/r -(yl-y)/r;
    (yl-y)/(r^2) -(xl-x)/(r^2);
];
zs = zeros(2, length(Sigma));
G = [zs(:, 1:landmarkID*2-2) g zs(:, landmarkID*2+1:end)];

K = Sigma*G'*(G*Sigma*G'+Q)^-1;

I = eye(length(Sigma));

h = [
    sqrt((xl+x)^2+(yl+y)^2)
    wrapToPi(atan2(yl-y, xl-x)-theta)
]';

update = (zi-h);
mu = mu + K*update';
Sigma = (I-K*G)*Sigma;
    
    
end
% -----------Add your functions below this line and use them in the two functions above---

