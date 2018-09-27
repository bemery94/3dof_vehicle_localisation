function points = getPointCloudFromSimRGBImage(depth_matrix, car_array, camera_array, cam_id)
    matlab_depth_matrix = flipud(depth_matrix);
    edge_image = car_array.image_array{cam_id};
    index = find(edge_image ~= 255);
    [row, col] = ind2sub(size(edge_image), index);
    
    edge_z_mm = matlab_depth_matrix(index);
    
    focal_length_pix = camera_array.getFocalLengthPix(cam_id);
    edge_x_mm = (col - camera_array.intrinsic_array{cam_id}.principal_point_x) .* edge_z_mm / focal_length_pix;
    edge_y_mm = (row - camera_array.intrinsic_array{cam_id}.principal_point_y) .* edge_z_mm / focal_length_pix;
    
    points_wrt_cam = [edge_x_mm, edge_y_mm, edge_z_mm, ones(size(edge_x_mm,1),1)];

    [r,c] = find(points_wrt_cam > 10000);
    points_wrt_cam(r,:) = [];
    
    % Convert point cloud to world frame
    % We have to account for the difference in camera frame between blender
    % and matlab i.e. blender has negative z axis going through camera. 
    % So:
    %   T_w_pcl = T_w_blender_cam * T_blender_cam_mat_cam * t_mat_cam_pcl
    % where w is world frame, pcl is the point cloud, blender_cam and
    % mat_cam are the coordinate frames of the cameras as mentioned above.
    % This gives us the point cloud in the world frame.
    T_w_cam = camera_array.extrinsic_array{cam_id};
    T_blender_cam_mat_cam = camera_array.T_blender_cam_mat_cam;
    points = (T_w_cam * T_blender_cam_mat_cam *  points_wrt_cam')';
end