classdef SimCarPoints
    properties
        points_wrt_world = {};
        points_wrt_camera = {};
    end
    
    methods
        function obj = untitled4(inputArg1,inputArg2)
            %UNTITLED4 Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = transformWorldPointsToCamera(self, T, cam_id)
            
        end
    end
end

