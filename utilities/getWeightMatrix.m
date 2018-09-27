% Returns the weight matrix based on the type of m estimator being used
% (huber or bisquare), the tuning constant and the size of the error
% vector. The weight matrix is used to weight each error value.

function W = getWeightMatrix(m_estimator_type, error, tuning_constant)
    error_size = size(error,1);
    w_vec = nan(error_size,1);
    for i=1:error_size
        if(abs(error(i)) <= tuning_constant)
            if (m_estimator_type == "huber")
                w_vec(i) = 1;
            elseif (m_estimator_type == "bisquare")
                w_vec(i) = (1 - (error(i) / tuning_constant)^2)^2;
            end
        else
            if (m_estimator_type == "huber")
                w_vec(i) = tuning_constant / abs(error(i));
            elseif (m_estimator_type == "bisquare")
                w_vec(i) = 0;
            end
        end
    end
    W = diag(w_vec);
end