classdef aruco < worldObject
    %ARUCO Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        TagNum
        ImgFile
        Corners
        Name
    end
    
    methods
        function obj = aruco(edgeLength, tagNum, position, orientation, identifier, parentFile, T_W2S)
            %CHECKERBOARD Construct an instance of this class 
            %   Detailed explanation goes here

             %Set up important variables
            %Body frame of aruco is positioned at top left corner of
            %top left block (black square. landscape). x goes right, y goes
            %down. z is into the board. Meanwhile, Actor has frame at
            %centre. Same x and y as Body, but z comes out of the board.

            T_W2S = T_W2S * [1 0 0 0; 0 -1 0 0; 0 0 1 0; 0 0 0 1];
            numBlocks = 6;

            R_A2B = [1 0 0;
                     0 1 0;
                     0 0 -1];
            t_A2B = [(edgeLength/2)*1000;
                     (edgeLength/2)*1000;
                      0]/1000;
            T_A2B = [R_A2B t_A2B; 0 0 0 1];

            %Calculate transformation from Body to World
            T_B2W = worldObject.calcTransformB2W(orientation, position);

             %Get transformatiion for Actor in Sim
            Rt_A2S = worldObject.calcTransformA2S(T_B2W, T_A2B, T_W2S);

            % %Calculate external dimensions
            dim = [edgeLength, edgeLength, 0.001];
            
            % %Create superclass instance
            obj = obj@worldObject(position, orientation, dim);
            

            %Set subclass properties
            obj.TagNum = tagNum;
            obj.ImgFile = aruco.generateArucoTagImg(parentFile, identifier, tagNum, edgeLength);

            obj.Corners = aruco.calcArucoEdgeCoords(edgeLength, T_B2W); 

            obj.Position = position;
            obj.Orientation = orientation;
            obj.Name = identifier;

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
        
        function imgFile = generateArucoTagImg(parentFile, name, id, targetSize)
            %Define some useful variables
            dpi = 80; %dots per inch
            dpmm = round((dpi/2.54), 0);%pixels per mm

            %Generate aruco image
            markerSize_pix = ceil(targetSize*1000*dpmm);
            markerFamily = "DICT_4x4_50";
            img = generateArucoMarker(markerFamily, id, markerSize_pix);
            img_f = flipdim(img ,2); 
            img_r = imrotate(img, 180);
            img_rgb = cat(3, img_r, img_r, img_r);


            %Save image
            targetFolder= fullfile(parentFile, 'Resources/aruco');
            targetName = strcat('texture_', name, '.png');
            imgFile = strcat(targetFolder, '\', targetName);
            imwrite(img_rgb, imgFile);

        end

        function corners = calcArucoEdgeCoords(edgeLength, T_B2W)

           
            %Some useful variables
            mmPerBlock = edgeLength/6;
            t_B2W = T_B2W(1:3,4);
            R_B2W = T_B2W(1:3,1:3);

            %In tag coordinate frame (body frame):
            crnr_topleft_B = [0; 0; -0.001];
            crnr_topright_B = [edgeLength; 0; -0.001];
            crnr_botleft_B = [0; edgeLength; -0.001];
            crnr_botright_B = [edgeLength; edgeLength; -0.001];

            %Convert to world frame
            crnr_topleft_W = (R_B2W *crnr_topleft_B) + t_B2W;
            crnr_topright_W = (R_B2W *crnr_topright_B) + t_B2W;
            crnr_botleft_W = (R_B2W *crnr_botleft_B) + t_B2W;
            crnr_botright_W = (R_B2W *crnr_botright_B) + t_B2W;
            
            corners = [crnr_topleft_W, crnr_topright_W, crnr_botright_W,  crnr_botleft_W]; %organised clockwise from top left

        end
    end
end

