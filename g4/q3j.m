function wheelVel = vw2wheels(vw, nolim)
    % heelVel = vw2wheels(vw) returns the wheel velocity vector as above, 
    % given the velocity and angular velocity as a 1x2 vector [v, w].
    
    % Inputs:
    % vw is the velocity vector (v, omega) in units of metres/s and radians/s

    % Wheel Vars.
    wheelDiameter = 0.065;
    lateralWheelSpacing = 0.16;
   
    % Seperating out the velocity vector for readability
    vel = vw(1);
    angVel = vw(2);
    
    % Return:
    % wheelVel is the wheel velocity vector (vleft, vright) each in the range -100 to +100 to achieve
    % this velocity
    
    % wheelVel = [VL VR];
    
    wheelVel = ([1 1; -1 1] \ [2 * vel; angVel * lateralWheelSpacing])';
    
    wheelVel = wheelVel / (pi * wheelDiameter * (1/60)) * 2;
    if nargin > 1
        if ~nolim
            if wheelVel(1) > 100
                wheelVel(1) = 100;
            elseif wheelVel(1) < -100
                wheelVel(1) = -100;
            end

            if wheelVel(2) > 100
                wheelVel(2) = 100;
            elseif wheelVel(2) < -100
                wheelVel(2) = -100;
            end
        end 
    end
    
end