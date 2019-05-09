% measurements probabilities = [p(z=1|sign) , p(z=1|no_sign)]
% z is the detector output. 1 sign, 0 no_sign
% signs_map : the map of where the signs are in the hallway.
measur_prob = [0.99,0.003];
signs_map = [0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0];
prior_belief   =  ones(1,length(signs_map) ) ./ length(signs_map);
z = 0;

%%
posterior_belief = discrete_localise1d(prior_belief,signs_map,measur_prob,z);
for c =1:length(posterior_belief)
    fprintf('The probability that the robot is in cell %d is %0.2f%s\n',c,posterior_belief(c)*100,'%')
end