function posterior_belief = discrete_localise1d(prior_belief,signs_map,measur_prob,z)
    % prior_belief: The prior belief of the robot about its location before incorporating any sensor information
    % signs_map : the map of where the signs are in the hallway.
    % measurements probabilities = [p(z=1|sign) , p(z=1|no_sign)]
    % z is the detector output. 1 sign, 0 no_sign.
    if z == 0
       measur_prob = 1 - measur_prob;
    end
    
    nominators = ones(length(signs_map), 1);
    for i = 1:length(signs_map)
        if signs_map(i) == 1
           nominators(i) = measur_prob(1) * prior_belief(i); 
        else
            nominators(i) = measur_prob(2) * prior_belief(i);
        end
    end
    
    denominator = 0;
    for i = 1:length(signs_map)
        if signs_map(i) == 1
            denominator = denominator + (measur_prob(1) * prior_belief(i)); 
        else
            denominator = denominator + (measur_prob(2) * prior_belief(i)); 
        end
    end
    
    posterior_belief = nominators/denominator;
end
