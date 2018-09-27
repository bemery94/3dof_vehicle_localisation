classdef SimCarImageArray < handle
    properties
        image_count;
        image_array;
        edge_array;
        corner_array;
        points_array;
    end
    
    methods
        function self = CarImageArray(image_path_list, display)   
            self.image_count = length(image_path_list);
            self.image_array = cell(self.image_count, 1);
            
            for i=1:self.image_count
                image_path = strcat(image_path_list(i).folder, '/', image_path_list(i).name);
                self.image_array{i} = imread(image_path);
                
                if display
                    figure;
                    imshow(self.image_array{i});
                end
            end
        end
        
        
        function [] = storeEdgeImages(self, display)
            self.edge_array = cell(self.image_count, 1);
            
            for i=1:self.image_count
                car_image = rgb2gray(self.image_array{i});
                edge_image = edge(car_image, 'Sobel');

%                 edge_image = imdilate(edge_image, strel('diamond', 3));
%                 edge_image = imerode(edge_image, strel('diamond', 2));

                self.edge_array{i} = edge_image;

                out_image = uint8(edge_image) * 255;

                if display
                    figure;
                    imshow(out_image);
                end
            end
        end
        
        function [] = storeCorners(self, corner_count, display)
            self.corner_array = cell(self.image_count, 1);
            
            for i=1:self.image_count
                corners = detectHarrisFeatures(self.edge_array{i});
                self.corner_array{i} = corners.selectStrongest(corner_count);
                
                if display
                    figure;
                    imshow(self.edge_array{i});
                    hold on;
                    plot(self.corner_array{i});
                end
            end
        end
        
        function [] = plot(self, i, plot_type)
            if strcmp(plot_type, 'image') || strcmp(plot_type, 'images')
                imshow(self.image_array{i});
            elseif strcmp(plot_type, 'edge') || strcmp(plot_type, 'edges')
                imshow(self.edge_array{i});
            elseif strcmp(plot_type, 'corner') || strcmp(plot_type, 'corners')
                plot(self.corner_array{i});
            end
        end
    end
end

