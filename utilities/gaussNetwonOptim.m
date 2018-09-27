%%%%
%
% Runs optimisation using Gauss-Newton method. 
%
%%%%

function [x, abs_mean_error, weighted_abs_mean_error] = gaussNetwonOptim(p_sim_filt, p_real_filt_1, camera_array, normals, iter_count, x_est)
    x = x_est;
    
    % Number of iterations of the optimisation to run for each set of point
    % correspondences.
    iter = 3;
    
    for i=1:iter
        % Calculate vector of error values according to custom cost 
        % function.
        [error, J, W] = calcErrorAndJacobian(x,  p_sim_filt, p_real_filt_1, ...
                                             camera_array, normals, iter_count);
                                         
        abs_mean_error = mean(abs(error));
        weighted_abs_mean_error = median(W*error);
        
        delta_x = (J' * W * J) \ (-J' * W) * error;
        x = x + delta_x;
    end
end