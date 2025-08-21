classdef april < worldObject
    %APRIL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        TagNum
        ImgFile
        Corners
    end
    
    methods
        function obj = april(edgeLength, tagNum, position, orientation, identifier, parentFile)
            %CHECKERBOARD Construct an instance of this class 
            %   Detailed explanation goes here

             %Set up important variables
            %Body frame of checkerboard is positioned at bottom right of
            %top left block (black square. landscape). x goes right, y goes
            %down. z is into the board. Meanwhile, Actor has frame at
            %centre. Same x and y as Body, but z comes out of the board.
            R_A2B = [1 0 0;
                     0 1 0;
                     0 0 -1];
            t_A2B = [(edgeLength/9)*(7/2);
                     (edgeLength/9)*(7/2);
                      0]/1000;
            T_A2B = [R_A2B t_A2B; 0 0 0 1];

            %Calculate transformation from Body to World
            T_B2W = worldObject.calcTransformB2W(orientation, position);

             %Get transformatiion for Actor in Sim
            Rt_A2S = worldObject.calcTransformA2S(T_B2W, T_A2B);

            % %Calculate external dimensions
            dim = [edgeLength, edgeLength, 0.001];
            
            % %Create superclass instance
            obj = obj@worldObject(position, orientation, dim);
            

            %Set subclass properties
            obj.TagNum = tagNum;
            obj.ImgFile = april.generate41h12TagImg(parentFile, identifier, tagNum, edgeLength);

            obj.Corners = april.calcAprilEdgeCoords(edgeLength, T_B2W); 

            orient_S_rad = (rotm2eul(Rt_A2S(1:3, 1:3), 'XYZ'));
            pos_S = transpose(Rt_A2S(1:3, 4));

            %Create 3D actor
            orientation_rad = deg2rad(orientation);
            obj.Actor = sim3d.Actor('ActorName', identifier, Translation=pos_S, Rotation=orient_S_rad);
            obj.Actor.Texture = obj.ImgFile;
            createShape(obj.Actor, 'plane', dim);


            
           
        end
    end
    methods (Static)
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end

        function imgFile = generate41h12TagImg(parentFile, name, number, targetSize)
            %Define some useful variables
            dpi = 80; %dots per inch
            dpmm = round((dpi/2.54), 0);%pixels per mm

            %Read april tag image from file
            sourceFile = fullfile('.', 'Resources\tagStandard41h12');
            fileName = strcat('tag41_12_', (num2str(number, '%05d')));
            sourceImg = imread(strcat(sourceFile, '\', fileName), 'png');
            
            %resize image to target size
            scaleFactor = (targetSize*1000*dpmm)/size(sourceImg, 1);
            resizedImg = imresize(sourceImg, scaleFactor, "nearest");
            texture = imrotate(resizedImg, -90);

            %Save image
            targetFolder= fullfile(parentFile, 'Resources');
            targetName = strcat('texture_', name, '.png');
            imgFile = strcat(targetFolder, '\', targetName);
            imwrite(texture, imgFile);

        end

        function corners = calcAprilEdgeCoords(edgeLength, Rt_B2W)

            %Some useful variables
            mmPerBlock = edgeLength/9;
            t_B2W = Rt_B2W(:,4);
            R_B2W = Rt_B2W(:,1:3);

            %In tag coordinate frame (body frame):
            crnr_topleft_B = [2*mmPerBlock; 2*mmPerBlock; -0.001];
            crnr_topright_B = [7*mmPerBlock; 2*mmPerBlock; -0.001];
            crnr_botleft_B = [2*mmPerBlock; 7*mmPerBlock; -0.001];
            crnr_botright_B = [7*mmPerBlock; 7*mmPerBlock; -0.001];

            %Convert to world frame
            crnr_topleft_W = (R_B2W *crnr_topleft_B) + t_B2W;
            crnr_topright_W = (R_B2W *crnr_topright_B) + t_B2W;
            crnr_botleft_W = (R_B2W *crnr_botleft_B) + t_B2W;
            crnr_botright_W = (R_B2W *crnr_botright_B) + t_B2W;
            
            corners = [crnr_topleft_W, crnr_topright_W, crnr_botleft_W,  crnr_botright_W];

        end
    end
end

