function [theta_out] = thetaTo360( theta_in )
%THETATO360 Summary of this function goes here
%   Detailed explanation goes here
    % Theta goes from 0 to 359,99999999...9
       
    theta_out = theta_in;
    
    % Theta goes from 0 to 359,99999999...9
    while sign(theta_out) == -1  
        theta_out = 360 + theta_out;
    end

end

