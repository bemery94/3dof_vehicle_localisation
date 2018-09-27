%%%%
%
% Manages properties of each camera in the camera array.
%
%%%%

classdef CameraArray < handle
    properties
        extrinsic_array = {};
        intrinsic_array = {};
        n;
        
        % Transform between the coordinate system of the camera in blender
        % and in matlab.
        T_blender_cam_mat_cam = trotx(pi);
    end
    
    methods
        function self = CameraArray(real_intrinsic_array, extrinsic_array)
            assert(size(extrinsic_array,2) == size(real_intrinsic_array,2), ...
                'Intrinsic and extrinsic arrays should be the same size.');
            self.n = size(extrinsic_array,2);
            self.extrinsic_array = extrinsic_array;
            self.intrinsic_array = real_intrinsic_array;
        end
        
        % Converts the focal length in mm to pixels.
        function focal_length_pix = getFocalLengthPix(self, cam_id)
            focal_length_pix = self.intrinsic_array{cam_id}.image_width_pix ...
                             * self.intrinsic_array{cam_id}.focal_length_mm ...
                             / self.intrinsic_array{cam_id}.sensor_width_mm;
        end
        
        function intrinsic_mat = getIntrinsicMatrix(self, cam_id)
            focal_length_pix = self.getFocalLengthPix(cam_id);
            principal_point_x = self.intrinsic_array{cam_id}.principal_point_x;
            principal_point_y = self.intrinsic_array{cam_id}.principal_point_y;
            intrinsic_mat = [focal_length_pix 0 principal_point_x;
                             0 focal_length_pix principal_point_y;
                             0 0 1];
        end
        
        % Project points into the image of the specified camera:
        %   [u v 1]' = K * T_cam_w * [I -C] * X
        function projected_points = projectPoints(self, points, cam_id)            
            T_blender_cam_w = invTform(self.extrinsic_array{cam_id});
            T_cam_mat_w = invTform(self.T_blender_cam_mat_cam) * T_blender_cam_w;
            R_cam_mat_w = T_cam_mat_w(1:3,1:3);
            t_cam_mat_w = T_cam_mat_w(1:3,4);

            intrinsic_mat = self.getIntrinsicMatrix(cam_id);

            projected_points = intrinsic_mat * [R_cam_mat_w t_cam_mat_w] * points;
            projected_points = projected_points(1:2,:) ./ projected_points(3,:);
        end
    end
end

