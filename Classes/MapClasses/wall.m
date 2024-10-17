classdef wall < worldObject
    %WALL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Colour
        
    end
    
    methods
        function obj = wall(position, orientation, dimensions, identifier, colour)
            %CHECKERBOARD Construct an instance of this class 
            %   Detailed explanation goes here

            obj = obj@worldObject(position, orientation, dimensions);
            
            %Set up important variables
            %Body frame of wall is positioned in the middle of the wall.
            %z-direction faces outwards, y faces down, x faces along the
            %wall. Actor is the same, but z faces inwards.
            R_A2B = [1 0 0;
                     0 1 0;
                     0 0 -1];
            t_A2B = [0; 0; 0];
            Rt_A2B = [R_A2B t_A2B];

            %Convert euler angles to rotation matrix
            Rt_B2W = worldObject.calcTransformB2W(orientation, position);

            %Get transformatiion for Actor in Sim
            Rt_A2S = worldObject.calcTransformA2S(Rt_B2W, Rt_A2B);

            orient_S_rad = (rotm2eul(Rt_A2S(1:3, 1:3), 'XYZ'));
            pos_S = transpose(Rt_A2S(1:3, 4));


            %Create 3D actor
            obj.Actor = sim3d.Actor('ActorName', identifier, Translation=pos_S, Rotation=orient_S_rad);
            createShape(obj.Actor, 'plane', dimensions);

            %Change colour
            obj.Colour = colour;
            obj.Actor.Color = colour;

        end
    end
    methods (Static)
    end
end

