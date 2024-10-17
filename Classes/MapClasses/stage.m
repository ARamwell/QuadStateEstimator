classdef stage < worldObject
    %STAGE Summary of this class goes here
    %Slightly different naming conventionm to other map objects. This is
    %because "floor.m" is a built-in MATLAB function.
    %   Detailed explanation goes here
    
    properties
        Colour
        
    end
    
    methods
        function obj = stage(position, orientation, dimensions, identifier, colour)
            %CHECKERBOARD Construct an instance of this class 
            %   Detailed explanation goes here

            Rt_A2B = [1 0 0 0; 0 1 0 0; 0 0 1 0];

            obj = obj@worldObject(position, orientation, dimensions);
            
            %Calculate transformation from Body to World
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

