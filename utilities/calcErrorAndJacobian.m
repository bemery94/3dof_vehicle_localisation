function [error,J,W] = calcErrorAndJacobian(x, p_sim_filt, p_real_filt_1, camera_array, normals, iter_count)
    theta = x(1);
    t = x(2:3);
    
    total_points = sum(cellfun('size', p_real_filt_1, 2));
    
    df_dtheta_without_norm = {};
    df_dt_without_norm = {};
       
    tx = t(1);
    ty = t(2);
    
    % Calculate the jacobian - Full details on this calculation are
    % available in my thesis report.
    for m=1:camera_array.n
        T_blender_cam_w = invTform(camera_array.extrinsic_array{m});
        T_cam_mat_w = invTform(camera_array.T_blender_cam_mat_cam) * T_blender_cam_w;
        R_cam_mat_w = T_cam_mat_w(1:3,1:3);
        t_cam_mat_w = T_cam_mat_w(1:3,4);
        intrinsic_mat = camera_array.getIntrinsicMatrix(m);

        r11 = R_cam_mat_w(1,1);
        r12 = R_cam_mat_w(1,2);
        r13 = R_cam_mat_w(1,3);
        r21 = R_cam_mat_w(2,1);
        r22 = R_cam_mat_w(2,2);
        r23 = R_cam_mat_w(2,3);
        r31 = R_cam_mat_w(3,1);
        r32 = R_cam_mat_w(3,2);
        r33 = R_cam_mat_w(3,3);
        lx = t_cam_mat_w(1);
        ly = t_cam_mat_w(2);
        lz = t_cam_mat_w(3);

        foc = intrinsic_mat(1,1);
        px = intrinsic_mat(1,3);
        py = intrinsic_mat(2,3);

        a = foc * r11 + px * r31;
        b = foc * r12 + px * r32;
        c = foc * r13 + px * r33;
        d = foc * lx + px * lz;
        e = foc * r21 + py * r31;
        f = foc * r22 + py * r32;
        g = foc * r23 + py * r33;
        h = foc * ly + py * lz;
        i = r31;
        j = r32;
        k = r33;
        l = lz;
        
        u_mat = [a*cos(theta) + b*sin(theta), -a*sin(theta) + b*cos(theta), c, a*tx + b*ty + d];
        v_mat = [e*cos(theta) + f*sin(theta), -e*sin(theta) + f*cos(theta), g, e*tx + f*ty + h];
        w_mat = [i*cos(theta) + j*sin(theta), -i*sin(theta) + j*cos(theta), k, i*tx + j*ty + l];
        
        u = (u_mat * p_sim_filt{m})';
        v = (v_mat * p_sim_filt{m})';
        w = (w_mat * p_sim_filt{m})';
        
        u_final = u ./ w;
        v_final = v ./ w;
        diff{m} = [u_final v_final] - p_real_filt_1{m}';
        
        % Calculate Jacobian
        du_dtheta = ((-a*sin(theta) + b*cos(theta))*p_sim_filt{m}(1,:) + (-a*cos(theta) - b*sin(theta))*p_sim_filt{m}(2,:))';
        du_dt = repelem([a b], size(p_sim_filt{m},2), [1 1]);
        
        dv_dtheta = ((-e*sin(theta) + f*cos(theta))*p_sim_filt{m}(1,:) + (-e*cos(theta) - f*sin(theta))*p_sim_filt{m}(2,:))';
        dv_dt = repelem([e f], size(p_sim_filt{m},2), [1 1]);
                
        dw_dtheta = ((-i*sin(theta) + j*cos(theta))*p_sim_filt{m}(1,:) + (-i*cos(theta) - j*sin(theta))*p_sim_filt{m}(2,:))';
        dw_dt = repelem([i j], size(p_sim_filt{m},2), [1 1]);
        
        % Of size [n,2], where each ith row is for the ith point. Col 1 is
        % d(u/w)/dtheta and col 2 is d(v/w)/dtheta.
        df_dtheta_without_norm{m} = [du_dtheta./w - (u .* dw_dtheta)./w.^2 dv_dtheta./w - (v.*dw_dtheta)./w.^2];
        
        % Of size [n,4], where each ith row is for the ith point. Col 1 is
        % d(u/w)/dtx and col 2 is d(u/w)/dty and col 3 is d(v/w)/dtx and
        % col 4 is d(v/w)/dty.
        df_dt_without_norm{m} = [du_dt./w - (u .* dw_dt)./w.^2 dv_dt./w- (v.*dw_dt)./w.^2];
    end

    normals_vec = horzcat(normals{:})';
    diff_vec = vertcat(diff{:});
    df_dtheta_without_norm_vec = vertcat(df_dtheta_without_norm{:});
    df_dt_without_norm_vec = vertcat(df_dt_without_norm{:});
    
    error = nan(total_points,1);
    error_size = size(error,1);
    J = nan(error_size, 3);
          
    for it=1:error_size
        error(it) = normals_vec(it,:) * diff_vec(it,:)';      
                       
        J(it,:) = normals_vec(it,:) * [df_dtheta_without_norm_vec(it,1:2)' reshape(df_dt_without_norm_vec(it,:), [2 2])'];  
    end
    
    % Here we set how often we want to recaculate the tuning constant based
    % on the median absolute deviation. I have found through experiments
    % that it works better to periodically recalculate this value. 
    %
    % Generally, it is better to start with huber and finish with bisquare.
    % Full details available in my thesis.
    persistent tuning_constant
    persistent rho_func
    if (iter_count == 1 || iter_count == 10)
        rho_func = "huber";
        MAD = medianAbsoluteDeviation(error);
        
        % Estimate of standard deviation
        sigma_hat = MAD / 0.6745;
        tuning_constant = 1.345 * sigma_hat;
    elseif (iter_count == 20 || iter_count == 30)
        rho_func = "bisquare";
        MAD = medianAbsoluteDeviation(error);
        
        % Estimate of standard deviation
        sigma_hat = MAD / 0.6745;
        tuning_constant = 4.6851 * sigma_hat;
    end
    W = getWeightMatrix(rho_func, error, tuning_constant);
end