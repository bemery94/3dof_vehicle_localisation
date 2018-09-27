%%%%
%
% Author: Brendan Emery (University of Technology Sydney, Australia.
% Email: b.emery94@gmail.com
%
% This is the main code developed for my capstone "3DoF Vehicle
% Localisation from Multi-View Camera Geometry". This code collects the
% real and simulated images from a specified directory and performs an
% optimisation to estimate the pose offset of the vehicle in the real
% images with respect to the simulated images.
%
%%%%

%%
clear;
close all;
addpath('utilities');

% Check if the robotics toolbox has been installed and correctly setup.
try
    rt2tr(eye(3), [0 0 0]');
catch
    error(['Make sure Peter Corke''s Robotics Toolbox is installed: ', ...
           'http://petercorke.com/wordpress/toolboxes/robotics-toolbox. ',...
           'If it has been installed, ensure that you''ve run the ', ...
           'startup_rvc.m script inside the toolbox.']);
end


%% Get real edges and sim edge point clouds

% plot_figures: Will plot the point correspondences found every n
% iterations. 
% show_raw_images: Shows the raw simulated and real edge images and the
% pointcloud extracted from the simulated edge images.
plot_figures = false;
show_raw_images = false;

% The cameras used in the blender simulation. This is used to get the name 
% of the directory containing the images so the values entered here must
% reflect the actual blender simulation. E.g. if images were rendered from
% cameras 1, 5 and 6, then this should be folder_cams = [1 5 6].
folder_cams = [3 4];

% The cameras that you want to use in the optimisation. Allows you to use a 
% subset of the cameras used in the blender simulation without rerunning
% the blender simulation. E.g. if the Blender simulation was captured using
% cameras 1, 5 and 6, you can select which images to use in the 
% optimisation by changing the values in the cam_id_list.
cam_id_list = [3 4];

% Ground truth offset in blender simulation.
x_gt = 0.2;
y_gt = 0.2;
theta_gt = 1;

% Get camera intrinsics, extrinsics and clipping distance for simulated
% cameras.
[camera_ext, camera_int, camera_depth_array] = getCameraParams(cam_id_list);

% Initialise cameras used in optimisation. These will only include the
% cameras set in cam_id_list.
camera_array = CameraArray(camera_int, camera_ext);

% Finds directory name for camera images from blender using the camera ids
% and the ground truth offset.
lower_dir = strcat('x_', num2str(x_gt,'%.2f'), '_y_', num2str(y_gt,'%.2f'), '_r_', num2str(theta_gt,'%.2f'));
upper_dir = 'cam_';
for cam_id=1:length(folder_cams)
    upper_dir = strcat(upper_dir, num2str(folder_cams(cam_id),'%02.f'), '_');
end
image_dir_sim = ['/home/brendan/capstone_handover_stuff/images/original_sim_pics/', upper_dir ,'/'];
image_dir_real = ['/home/brendan/capstone_handover_stuff/images/original_real_pics/', upper_dir, '/', lower_dir ,'/'];

% Get names of each individual image
image_list_sim = {};
image_list_real = {};
for i=1:length(cam_id_list)
    image_list_sim{i} = [image_dir_sim, 'image/camera_', num2str(cam_id_list(i),'%02.f'),'.png'];
    image_list_real{i} = [image_dir_real, 'image/camera_', num2str(cam_id_list(i),'%02.f'),'.png'];
end

% Store the camera images
image_count_sim = length(image_list_sim);
image_count_real = length(image_list_real);
car_array_sim = CarImageArray(image_list_sim);
car_array_real = CarImageArray(image_list_real);

% Extract edges from real camera images and then find indices of each pixel
% along edges.
car_array_real.storeEdgeImages();
car_array_real.storePoints();

% Get point cloud from each camera in world frame.
for m=1:length(cam_id_list)
    load(image_dir_sim + "depth/" + "depth_" + num2str(cam_id_list(m),'%02.f') + ".mat");
    sim_car_points.points_wrt_world{m} = getPointCloudFromSimFreestyleImage(depth_matrix, car_array_sim, camera_array, m, camera_depth_array);
end

%% Show edge images and pointcloud generated from each camera.
if (show_raw_images)
    for m=1:camera_array.n
        scatter3(sim_car_points.points_wrt_world{m}(:,1), sim_car_points.points_wrt_world{m}(:,2), ...
                 sim_car_points.points_wrt_world{m}(:,3),'.');
        title('Edge point cloud in world frame');
        daspect([1 1 1])
        figure;
        car_array_sim.plot(m, 'image');
        title('Sim edge image');
        figure;
        car_array_real.plot(m, 'edges');
        title('Real edge image');
        pause;
        close all;
    end
end

%% Optimization
% Initial guess
theta_est = deg2rad(0);
t_est = [0.0 0.0]';

% Get 3d simulated points and 2d real image points. Use cellfun to
% transpose the matrix of points for each camera.
p_sim = cellfun(@transpose,sim_car_points.points_wrt_world,'UniformOutput',false);
p_real = cellfun(@transpose,car_array_real.point_array,'UniformOutput',false); 

% Threshold used for trimming largest correspondence distances. This
% currently isn't used (min_overlap_thresh of 1.0 means no correspondences
% are trimmed) as m-estimation works better at dealing with outliers.
min_overlap_thresh = 1.0;

normals = cell(1,camera_array.n);
delta_error = inf;
iter_count = 1;

abs_mean_error_vec = [];
weighted_abs_mean_error_vec = [];
delta_error_vec = [];

% The optimisation can either be run for a set number of iterations or
% until the change in error between iterations is below a threshold.
while iter_count < 40 %(delta_error > 5e-5)
    
    % Calculate point correspondences for each camera.
    for m=1:camera_array.n    
        % Transform points from simulated point cloud to real image frame, 
        % same transformation for each camera.
        T_k = rt2tr(rotz(theta_est), [t_est(1), t_est(2), 0]');
        p_sim_tformed = T_k * p_sim{m};

        % Project points into image:
        %   [u v 1]' = K * T_cam_w * [I -C] * X
        p_sim_tformed_proj = camera_array.projectPoints(p_sim_tformed, m);

        % Find correspondences between projected points and real image
        % points for each camera.
        [idx, distances] = knnsearch(p_real{m}', p_sim_tformed_proj', 'k', 2);

        % Calculate average distances of the distances between the point
        % and its 2 matches.
        average_distances = mean(distances, 2);

        % Sort matches by distance and only take the shortest distances.
        [sorted_distances, sorted_idx] = sort(average_distances,'ascend');
        sorted_idx = sorted_idx(1:size(sorted_idx,1)*min_overlap_thresh,:);
        idx = [idx [1:size(idx,1)]'];
        idx = idx(sorted_idx,:);

        % Get the points from the real and projected sim images with the
        % shortest distances between point correspondences.   
        % Each column of p_sim_tformed_proj_filt is a single point in the sim data set
        % Each column in p_real_filt_1 is the closest neighbour to the point in the sim
        %   data set in the same column.
        % Each column in p_real_filt_2 is the 2nd closest neighbour to the point in the 
        %   sim data set in the same column.
        p_real_filt_1{m} = p_real{m}(:, idx(:,1));
        p_real_filt_2{m} = p_real{m}(:, idx(:,2));
        p_sim_tformed_proj_filt = p_sim_tformed_proj(:, idx(:,3));
        p_sim_filt{m} = p_sim{m}(:, idx(:,3));

        % Plots point correspondences for visualisation.
        if (plot_figures && mod(iter_count,10)==1)
            figure;
            h1=axes;
            scatter(p_sim_tformed_proj(1,:), p_sim_tformed_proj(2,:), 50,'g','x');
            hold on
            scatter(p_real{m}(1,:), p_real{m}(2,:),10,'r','x');
            hold on        
            title('Nearest Neighbour Point Correspondences');                    
            set(h1, 'Ydir', 'reverse');

            h.PaperUnits = 'inches';
            h.PaperPosition = [0 0 18 14];

            plot_p_real_filt = p_sim_tformed_proj_filt';
            plot_p_sim_proj_filt_1 = p_real_filt_1{m}';
            plot_p_sim_proj_filt_2 = p_real_filt_2{m}';
            plot([plot_p_real_filt(:, 1) plot_p_sim_proj_filt_1(:,1)]', ...
                 [plot_p_real_filt(:, 2) plot_p_sim_proj_filt_1(:,2)]','b');
            plot([plot_p_real_filt(:, 1) plot_p_sim_proj_filt_2(:,1)]', ...
                 [plot_p_real_filt(:, 2) plot_p_sim_proj_filt_2(:,2)]','m');

            % Option to save the figure
            %    print(['output_images/original_point_correspondences/points_correspondence_cam_', num2str(m), '_im_', num2str(iter_count),'.png'],'-dpng','-r100');
            pause;
            close all; 
        end
    end

    % For each correspondence between each simulated edge point and the two
    % real edge points, calculate the normal to the line joining the two
    % real edge points.
    for m=1:camera_array.n
        normals{m} = [];
        for i=1:size(p_real_filt_1{m},2)
            norm = calculateNorm(p_real_filt_1{m}(:,i), p_real_filt_2{m}(:,i));
            normals{m} = [normals{m} norm];
        end
    end
    
    % Run iteration of optimisation
    [x, abs_mean_error, weighted_abs_mean_error] = gaussNetwonOptim(p_sim_filt, p_real_filt_1, camera_array, normals, iter_count, [theta_est; t_est]);
    
    % Various metrics to evaluate optimisation
    abs_mean_error_vec = [abs_mean_error_vec; abs_mean_error];
    weighted_abs_mean_error_vec = [weighted_abs_mean_error_vec; weighted_abs_mean_error];

    delta_error = sqrt((x(1) - theta_est)^2 + (x(2) - t_est(1))^2 + (x(3) - t_est(2))^2)
    delta_error_vec = [delta_error_vec; [abs(deg2rad(theta_gt) - x(1)), abs(x_gt - x(2)), abs(y_gt - x(3))]];

    print_x = [rad2deg(x(1)); x(2:3)];
    fprintf('Estimated (deg, m, m): %s\n', sprintf('%f ', print_x));

    theta_est = x(1);
    t_est = x(2:3);

    iter_count = iter_count + 1
end

disp(['Number of iterations = ', num2str(iter_count)]);

%% Plot final figures
for cam_id=1:length(cam_id_list)
    cam = cam_id_list(cam_id);
    
    % Transform point cloud given current guess. 
    x = [theta_est; t_est];
    
    test_T = rt2tr(rotz(x(1)), [x(2), x(3), 0]');
    
    test_p_t_3d = test_T * sim_car_points.points_wrt_world{cam_id}';

    % Project points into image:
    %   [u v 1]' = K * T_cam_w * [I -C] * X
    test_p_t_projected = camera_array.projectPoints(test_p_t_3d, cam_id);
    
    FigH1 = figure('Position', get(0, 'Screensize'));
    h1 = axes;

    scatter(p_real{cam_id}(1,:), p_real{cam_id}(2,:), 50,'r','.');
    hold on
    p_i_proj = camera_array.projectPoints(p_sim{cam_id}, cam_id);
    scatter(p_i_proj(1,:), p_i_proj(2,:),20,'g','x');
    scatter(test_p_t_projected(1,:), test_p_t_projected(2,:),40,'b','x');
    hold on 
    
    title(['Optimisation Results. Cam ', num2str(cam)], 'fontsize',38);
    set(h1, 'Ydir', 'reverse');
    [hleg, hobj, hout, mout] = legend({'Points from real image', 'Projected points from simulated model', 'Transformed points using optimised rotation and translation'}, 'fontsize', 22,'Location','southoutside');
    hobj(4).Children.MarkerSize = 25;
    hobj(5).Children.MarkerSize = 15;
    hobj(6).Children.MarkerSize = 15;
 
    set(gca,'xtick',[])
    set(gca,'ytick',[])
    daspect([1 1 1])
    
    str = 'Points from real image';
    str2 = 'Projected points from simulated model';
    str3 = 'Transformed points using optimised rotation and translation';
    
    F    = getframe(FigH1);
end






