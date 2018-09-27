%%%%
% Script to convert the images in *.bin format into usable/viewable
% *.png files.
%
% Some of the images provided from the real factories (particularly the
% superimposed images which superimpose the images of the vehicle as the 
% light array moves across the vehicle.
%%%%
clear
close all

path = '/home/brendan/ernesto_data/Stop2/';
image_list = dir(strcat(path, 'CAM*_micros.bin'));

for i=1:length(image_list)
    f=fopen(strcat(image_list(i).folder, '/', image_list(i).name));
    Img = fread(f,[2592 2048],'float32');
    Img=Img';
    Img=imresize(uint8(Img./150), 0.5);
    fclose(f);
    imwrite(Img, ['/home/brendan/google_drive/autumn_2018/capstone_B/final_report/code/matlab/utilities/ernesto_data/image/camera_', num2str(i), '.png']);
%     figure
%     imshow(uint8(Img./150));
%     pause
end
