% Function that returns the extrinsics, intrinsics and maximum camera depth
% of each camera. These are currently set to the Blender simulation
% provided of the cameras in the car manufacturing plant. 
%
% The camera depth is the clipping distance for the simulated cameras. This
% is manually chosen to allow us to choose how much of the vehicle each
% simulated camera sees. E.g. it is currently set so that most cameras only
% see the front face of the vehicle, and don't see the interior of the
% vehicle.
%
% Note. The output resolution of the real camera images and blender
% simulation images should be the same. The image height and image width
% needs to be updated in the camera intrinsics.

function [camera_ext, camera_int, camera_depth] = getCameraParams(cam_id_list)
    % Camera extrinsics taken from the blender simulation.
    cameras = {};
    cameras{1} = rt2tr(quat2rotm([-0.668558 -0.274251 0.324338 0.610427]), [-2.76769 -0.296352 2.62169]');
    cameras{2} = rt2tr(quat2rotm([-0.610426 -0.324337 0.27425 0.66856]), [-2.76769 0.296352 2.62169]');
    cameras{3} = rt2tr(quat2rotm([-0.66635 -0.629863 0.159071 0.365987]), [-2.72129 -2.00223 1.12694]');
    cameras{4} = rt2tr(quat2rotm([-0.365987 -0.159072 0.629863 0.66635]), [-2.72129 2.00223 1.12694]');
    cameras{5} = rt2tr(quat2rotm([-0.344277 -0.632524 -0.323817 0.613619]), [-0.788036 -2.04755 0.478405]');
    cameras{6} = rt2tr(quat2rotm([-0.613504 0.323605 0.632634 0.344481]), [-0.788036 2.04755 0.478405]');
    cameras{7} = rt2tr(quat2rotm([-0.745457 -0.611835 0.184114 0.18988]), [-0.287872 -2.01047 1.12815]');
    cameras{8} = rt2tr(quat2rotm([-0.189879 -0.184113 0.611833 0.745458]), [-0.287872 2.01047 1.12815]');
    cameras{9} = rt2tr(quat2rotm([-0.606398 -0.747698 0.162409 0.216456]), [0.114018 -2.07243 0.075371]');
    cameras{10} = rt2tr(quat2rotm([-0.216456 -0.162408 0.747698 0.606398]), [0.114018 2.07243 0.075371]');
    cameras{11} = rt2tr(quat2rotm([0.647411 0.724867 0.202231 0.120538]), [2.11154 -2.07788 0.091279]');
    cameras{12} = rt2tr(quat2rotm([0.120538 0.202231 0.724867 0.647411]), [2.11154 2.07788 0.091279]');
    cameras{13} = rt2tr(quat2rotm([0.724192 0.637276 0.187178 0.185446]), [2.12527 -2.03324 1.0296]');
    cameras{14} = rt2tr(quat2rotm([0.185446 0.187178 0.637276 0.724192]), [2.12527 2.03324 1.0296]');
    cameras{15} = rt2tr(quat2rotm([0.599856 0.757972 0.115841 0.228543]), [3.26644 -2.0687 0.06874]');
    cameras{16} = rt2tr(quat2rotm([0.228543 0.115841 0.757973 0.599855]), [3.26644 2.0687 0.06874]');
    cameras{17} = rt2tr(quat2rotm([0.654975 0.687262 0.066097 0.307098]), [3.29027 -2.01712 1.02942]');
    cameras{18} = rt2tr(quat2rotm([0.307098 0.066097 0.687262 0.654975]), [3.29027 2.01712 1.02942]');
    cameras{19} = rt2tr(quat2rotm([-0.929859 -0.252809 0.267216 0.006737]), [0.781395 -0.989112 2.43351]');
    cameras{20} = rt2tr(quat2rotm([-0.006737 -0.267214 0.252807 0.92986]), [0.781395 0.989112 2.43351]');
    cameras{21} = rt2tr(quat2rotm([-0.964039 -0.225759 0.137783 0.026021]), [1.84795 -0.989007 2.4297]');
    cameras{22} = rt2tr(quat2rotm([-0.026021 -0.137782 0.225757 0.96404]), [1.84795 0.989007 2.4297]');
    cameras{23} = rt2tr(quat2rotm([0.67438 0.212631 0.212631 0.67438]), [3.7165 0.000001 2.64653]');

    % Camera intrinsics
    image_width = 1036;
    image_height = 819;
    camera_intrinsics = {};
    camera_intrinsics{1} = struct('principal_point_x', image_width/2, ...
                          'principal_point_y', image_height/2, ...
                          'image_width_pix', image_width, ...
                          'sensor_width_mm', 12.44, ...
                          'focal_length_mm', 35.0);
    camera_intrinsics{2} = camera_intrinsics{1};
    camera_intrinsics{3} = struct('principal_point_x', image_width/2, ...
                          'principal_point_y', image_height/2, ...
                          'image_width_pix', image_width, ...
                          'sensor_width_mm', 12.44, ...
                          'focal_length_mm', 50.0);
    camera_intrinsics{4} = camera_intrinsics{3};
    camera_intrinsics{5} = struct('principal_point_x', image_width/2, ...
                          'principal_point_y', image_height/2, ...
                          'image_width_pix', image_width, ...
                          'sensor_width_mm', 12.44, ...
                          'focal_length_mm', 12.5);
    camera_intrinsics{6} = camera_intrinsics{5};
    camera_intrinsics{7} = camera_intrinsics{6};
    camera_intrinsics{8} = camera_intrinsics{7};
    camera_intrinsics{9} = camera_intrinsics{8};
    camera_intrinsics{10} = camera_intrinsics{9};
    camera_intrinsics{11} = camera_intrinsics{10};
    camera_intrinsics{12} = camera_intrinsics{11};
    camera_intrinsics{13} = camera_intrinsics{12};
    camera_intrinsics{14} = camera_intrinsics{13};
    camera_intrinsics{15} = camera_intrinsics{14};
    camera_intrinsics{16} = camera_intrinsics{15};
    camera_intrinsics{17} = camera_intrinsics{16};
    camera_intrinsics{18} = camera_intrinsics{17};
    camera_intrinsics{19} = camera_intrinsics{18};
    camera_intrinsics{20} = camera_intrinsics{19};
    camera_intrinsics{21} = struct('principal_point_x', image_width/2, ...
                          'principal_point_y', image_height/2, ...
                          'image_width_pix', image_width, ...
                           'sensor_width_mm', 12.44, ...
                           'focal_length_mm', 10.0);
    camera_intrinsics{22} = camera_intrinsics{21};
    camera_intrinsics{23} = struct('principal_point_x', image_width/2, ...
                          'principal_point_y', image_height/2, ...
                          'image_width_pix', image_width, ...
                           'sensor_width_mm', 12.44, ...
                           'focal_length_mm', 8.0);

    % Camera depths
    camera_depths = {};
    camera_depths{1} = 4;
    camera_depths{2} = 4;
    camera_depths{3} = 4;
    camera_depths{4} = 4;
    camera_depths{5} = 2;
    camera_depths{6} = 2;
    camera_depths{7} = 2.5;
    camera_depths{8} = 2.5;
    camera_depths{9} = 2.4;
    camera_depths{10} = 2.5;
    camera_depths{11} = 2;
    camera_depths{12} = 2;
    camera_depths{13} = 2;
    camera_depths{14} = 2;
    camera_depths{15} = 2.3;
    camera_depths{16} = 2.3;
    camera_depths{17} = 2;
    camera_depths{18} = 2;
    camera_depths{19} = 2.9;
    camera_depths{20} = 2.9;
    camera_depths{21} = 2.0;
    camera_depths{22} = 2.0;
    camera_depths{23} = 25.0;
    
    % Return the intrinsics, extrinsics and camera depths for the cameras
    % that have been chosen for use in the optimisation.
    camera_ext = {}; 
    camera_int = {};
    camera_depth = {};
    for cam=1:length(cam_id_list)
        camera_ext{cam} = cameras{cam_id_list(cam)};
        camera_int{cam} = camera_intrinsics{cam_id_list(cam)};
        camera_depth{cam} = camera_depths{cam_id_list(cam)};
    end
end








