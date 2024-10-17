classdef checker < worldObject
    %CHECKERBOARD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        NumSquares
        SquareSize
        Corners
        ImageFile
    end
    
    methods
        function obj = checker(checkerSize, squareSize, position, orientation, identifier)
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
            %t_A2B = [(squareSize*(checkerSize(2)-2)/2);
            %         (squareSize*(checkerSize(1)-2)/2);
            %          0];
            t_A2B = [0;0;0];
            Rt_A2B = [R_A2B t_A2B];

            % %Calculate external dimensions of board
            height = squareSize*checkerSize(1);
            width = squareSize*checkerSize(2);
            dim = [width/1000, height/1000, 0.000];

            %Calculate transformation from Body to World
            Rt_B2W = worldObject.calcTransformB2W(orientation, position);

            
            % %Create superclass instance
            %dim = 1;
            obj = obj@worldObject(position, orientation, dim);
            

            %Set subclass properties
            obj.NumSquares = checkerSize;
            obj.SquareSize = squareSize;

            %Get transformatiion for Actor in Sim
            Rt_A2S = worldObject.calcTransformA2S(Rt_B2W, Rt_A2B);

            orient_S_rad = (rotm2eul(Rt_A2S(1:3, 1:3), 'XYZ'));
            pos_S = transpose(Rt_A2S(1:3, 4))/1000;

            obj.Corners = checker.calcCheckerEdgeCoords_W(checkerSize, squareSize, Rt_B2W); 

            obj.ImageFile = checker.generateCheckerboardImg(checkerSize, squareSize, identifier);

            %Create 3D actor
            orientation_rad = deg2rad(orientation);
            obj.Actor = sim3d.Actor('ActorName', identifier, Translation=pos_S, Rotation=orient_S_rad);
            obj.Actor.Texture = obj.ImageFile;
            createShape(obj.Actor, 'plane', dim);
            


            
           
        end
    end
    methods (Static)
        

        function imgFile = generateCheckerboardImg(checkerSize, squareSize, identifier)
            %Create checkerboard
             
            %Define some useful variables
            dpi = 80; %dots per inch
            dpmm = round((dpi/2.54), 0);%pixels per mm

            doubleboard = checkerboard(squareSize, checkerSize(1), checkerSize(2)) >0.5;
            %board = doubleboard(square_size+1:(square_size*(num_squares(1)+1)), 1:(square_size*num_squares(2)));
            board = doubleboard(1:(squareSize*(checkerSize(1))), 1:(squareSize*checkerSize(2)));
            
            
            %approx_checkerboard_dim_pixels = (checkerSize * squareSize);
            %approx_checkerboard_dim_fractional = approx_checkerboard_dim_pixels/canvas_size;
            %approx_checkerboard_dim = approx_checkerboard_dim_fractional * P_x * 100;

            
            scaleFactor = (dpmm);
            board_hires = imresize(board, scaleFactor, "nearest");
           
            
            %texture = padarray(board, padding_size, 1, 'both');
            texture = (board_hires*254);
            texture = transpose(texture);
            texture = imrotate(texture, 90);
            texture = cat(3, texture, texture, texture); %rgb

            imgFile = strcat("C:/Users/alyss/OneDrive - University of Cape Town/Sandbox/PnP/Pnp_solver/GitClone/Resources/checkerboardTexture_", string(identifier), ".png");
            
            imwrite(texture, imgFile);
            
            %disp("Checkerboard finished")
            %disp("Approximate checkerboard dimensions")
            %disp(approx_checkerboard_dim + "cm")
        end

        function corners = calcCheckerEdgeCoords_W(checkerSize, squareEdgeLength, Rt_B2W)
        
            boardWidth = checkerSize(2);
            boardHeight = checkerSize(1);
            X_pnts_W = zeros(3,(boardWidth-1)*(boardHeight-1));
            X_pnts_B = zeros(3,(boardWidth-1)*(boardHeight-1));
        
            R_B2W = Rt_B2W(1:3,1:3);
            t_B2W = Rt_B2W(1:3,4)*1000;

            x_offset = ((boardWidth-2)*squareEdgeLength)/2;
            y_offset = ((boardHeight-2)*squareEdgeLength)/2;
        
            %for each column
            for c=1 : boardWidth-1
                for r = 1: boardHeight-1
                    x = (c-1)*squareEdgeLength-x_offset;
                    y = (r-1)*squareEdgeLength-y_offset;
                    z = -1;
                    X_pnts_B(1,  (((c-1)*(boardHeight-1)) + r)) = x;
                    X_pnts_B(2,  (((c-1)*(boardHeight-1)) + r)) = y;
                    X_pnts_B(3,  (((c-1)*(boardHeight-1)) + r)) = z;
                end
            end

            %For each point
            for i=1:size(X_pnts_B,2)
                X_B = X_pnts_B(:,i); 
                X_pnts_W(:,i) = (R_B2W * X_B) + t_B2W;
            end

            corners = X_pnts_W;
        end
    end
end

