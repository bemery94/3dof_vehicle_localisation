%%%%
%
% Class to store the camera images, edge images extracted from those images 
% and indices of points along each of those edges. 
%
%%%%

classdef CarImageArray < handle
    properties
        image_count
        image_array
        edge_array
        point_array
    end
    
    methods
        function self = CarImageArray(image_path_list)   
            self.image_count = length(image_path_list);
            self.image_array = cell(self.image_count, 1);
            
            for i=1:self.image_count
                self.image_array{i} = imread(image_path_list{i});
            end
        end
        
        % Dilate and erode the edge images to try and produce more
        % continuous edges.
        function [] = dilateAndErodeFreestyleImages(self)
            for i=1:self.image_count
                image = self.image_array{i};
                
                image = imcomplement(image);
                
                image = imdilate(image, strel('diamond', 10));
                image = imerode(image, strel('diamond', 10));
                
                image = imcomplement(image);
                
                self.image_array{i} = image;                
            end
        end
        
        function [] = storeEdgeImages(self)
            self.edge_array = cell(self.image_count, 1);
            
            for i=1:self.image_count
                % If the image isn't already grayscale, convert it.
                %    car_image = rgb2gray(self.image_array{i});
                car_image = self.image_array{i};
                edge_image = edge(car_image, 'Sobel', 0.04);

                % Dilation and erosion can be used improve edge continuity.
                % This will likely be needed when using real camera images,
                % but for simulated camera images it's not needed.
                %    edge_image = imdilate(edge_image, strel('diamond', 3));
                %    edge_image = imerode(edge_image, strel('diamond', 2));

                self.edge_array{i} = edge_image;
                out_image = uint8(edge_image) * 255;
            end
        end
        
        % Store the indices of each point along edges.
        function [] = storePoints(self)
            self.point_array = cell(self.image_count, 1);
            
            for i=1:self.image_count
                edge_image = self.edge_array{i};
                index = find(edge_image);
                [row, col] = ind2sub(size(edge_image), index);
                
                self.point_array{i} = [col row];
            end
        end
        
        function [] = plot(self, i, plot_type)
            if strcmp(plot_type, 'image') || strcmp(plot_type, 'images')
                image = self.image_array{i};
                image_comp = imcomplement(image);
                imshow(image_comp);
            elseif strcmp(plot_type, 'edge') || strcmp(plot_type, 'edges')
                imshow(self.edge_array{i});
            elseif strcmp(plot_type, 'point') || strcmp(plot_type, 'points')
                points = self.point_array{i};
                scatter(points(:,1), points(:,2));
            end
        end
    end
end

