% Convert simulated Blender images and corresponding depth values into a
% pointcloud in the world frame.

function points = getPointCloudFromSimFreestyleImage(depth_matrix, car_array, camera_array, cam_id, camera_depth_array)
    % Flip the matrix since the y axis of matlab images points down.
    matlab_depth_matrix = flipud(depth_matrix);
    edge_image = car_array.image_array{cam_id};
    
    % Find every pixel in the edge image that is not white (these are the
    % edge pixels) and convert to row and column indices.
    index = find(edge_image ~= 255);
    [row, col] = ind2sub(size(edge_image), index);
    
    % Get the depth value in mm for each edge pixel
    edge_z_mm = matlab_depth_matrix(index);
    
    % Convert the x and y coordinates of each edge pixel from image
    % coordinates into real world units (mm) taken with respect to the 
    % camera.
    focal_length_pix = camera_array.getFocalLengthPix(cam_id);
    edge_x_mm = (col - camera_array.intrinsic_array{cam_id}.principal_point_x) .* edge_z_mm / focal_length_pix;
    edge_y_mm = (row - camera_array.intrinsic_array{cam_id}.principal_point_y) .* edge_z_mm / focal_length_pix;
    points_wrt_cam = [edge_x_mm, edge_y_mm, edge_z_mm, ones(size(edge_x_mm,1),1)];

    % Pixels in the image which do not lie on an object (in this case the
    % vehicle) will have depth values of ~1e10. We find these pixels and
    % remove them.
    [r,c] = find(points_wrt_cam > 10000);
    points_wrt_cam(r,:) = [];
    
    % Remove pixels which have distance values larger than the clipping
    % distance of the camera.
    [r2,c2] = find(points_wrt_cam(:,3) > camera_depth_array{cam_id});
    points_wrt_cam(r2,:) = [];
    
    % Convert point cloud to world frame
    % We have to account for the difference in camera frame between blender
    % and matlab i.e. blender has negative z axis going through camera. 
    % So:
    %   T_w_pcl = T_w_blender_cam * T_blender_cam_mat_cam * t_mat_cam_pcl
    % where w is world frame, pcl is the point cloud, blender_cam and
    % mat_cam are the coordinate frames of the cameras as mentioned above.
    % This gives us the point cloud in the world frame. t_mat_cam_pcl is
    % the point cloud with respect to the matlab camera frame. It's in the
    % matlab camera frame since we use matlab array indexing (row,col) in
    % finding edge_x_mm and edge_y_mm.
    T_w_cam = camera_array.extrinsic_array{cam_id};
    T_blender_cam_mat_cam = camera_array.T_blender_cam_mat_cam;
    points = (T_w_cam * T_blender_cam_mat_cam *  points_wrt_cam')';
end