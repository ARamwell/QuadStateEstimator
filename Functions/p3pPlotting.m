classdef p3pPlotting

    methods (Static)

        %------------------------------------------------------------%
        function axObj = initPlot(figObj, xLim, yLim, zLim)
            %XLim, etc are 1x2 arrays, e.g. [0 1000].

            axObj = axes('Parent', figObj);
            axis equal;
            xlim(axObj, xLim);
            ylim(axObj,yLim);
            zlim(axObj,zLim);
            xlabel(axObj, 'x (mm)');
            ylabel(axObj, 'y (mm)');
            zlabel(axObj, 'z (mm)');
            %set(trajPlotAx, 'Ydir', 'reverse');
            view(axObj, 3);
            grid(axObj, 'on');
            hold(axObj, 'on');
        end

        %------------------------------------------------------------%

        % function [lineObj, textObjX, textObjY, textObjZ,  lineObjX,  lineObjY,  lineObjZ] = addTraj(axObj, name)
        function [lineObj, arr_axesTextObj, arr_axesLineObj] = addTraj(axObj, name, lineStyle_traj, lineStyle_coordAx)
        %Add a new trajectory line to the existing plot
            
            %Add trajectory line
            lineObj = plot3(axObj, [NaN],[NaN],[NaN], lineStyle_traj, 'DisplayName', name);
            
            %Add coordinate axes
            blankCoords = [NaN NaN; NaN NaN; NaN NaN];
            lineObjX = plot(axObj, blankCoords, lineStyle_coordAx);
            lineObjY =plot(axObj, blankCoords, lineStyle_coordAx);
            lineObjZ =plot(axObj, blankCoords, lineStyle_coordAx);

            %Add coordinate axes labels
            textObjX = text(axObj, NaN, NaN, NaN, 'x');
            textObjY = text(axObj, NaN, NaN, NaN, 'y');
            textObjZ = text(axObj, NaN, NaN, NaN, 'z');

            arr_axesTextObj = [textObjX textObjY textObjZ];
            arr_axesLineObj = [lineObjX lineObjY lineObjZ];


        end

        %------------------------------------------------------------%

        function  scatterObj = addCheckerboard(axObj, cornerPnts)
            %Corner points imported as array of column vectors
            
            scatterObj = scatter3(axObj,transpose(cornerPnts(1,:)), transpose(cornerPnts(2,:)),transpose(cornerPnts(3,:)), 4, "black", "filled");


        end

        %------------------------------------------------------------%

        function updateTraj(lineObj, arr_axTextObj, arr_axLineObj, rtHist)

            tHist_x(:) =rtHist(1,4,:);
            tHist_y(:) =rtHist(2,4,:);
            tHist_z(:) =rtHist(3,4,:);

            %Extract final point
            t = rtHist(:,4,end);
            R = rtHist(:,1:3,end);
            
            coordAxEnd = t + (R* [100 0 0; 0 100 0; 0 0 100]);

            xAxData = [t coordAxEnd(:,1)];
            yAxData = [t coordAxEnd(:,2)];
            zAxData = [t coordAxEnd(:,3)];

            %Update trajectory line
            set(lineObj, 'XData',  tHist_x, 'YData', tHist_y, 'ZData', tHist_z);
            
            %Update coordinate axes (axis lines)
            set(arr_axLineObj(1), 'XData',  xAxData(1,1:2), 'YData', xAxData(2,1:2), 'ZData', xAxData(3,1:2));
            set(arr_axLineObj(2), 'XData',  yAxData(1,1:2), 'YData', yAxData(2,1:2), 'ZData', yAxData(3,1:2));
            set(arr_axLineObj(3), 'XData',  zAxData(1,1:2), 'YData', zAxData(2,1:2), 'ZData', zAxData(3,1:2));

            %Update coordinate axes (text position)
            set(arr_axTextObj(1), 'Position', transpose(coordAxEnd(:,1)));
            set(arr_axTextObj(2), 'Position', transpose(coordAxEnd(:,2)));
            set(arr_axTextObj(3), 'Position', transpose(coordAxEnd(:,3)));

            

        end

        %------------------------------------------------------------%
        function plotImagePlaneOutline(Ax, imgRes, Rt)
        
        
        end


       %------------------------------------------------------------%
        function plotImagePlanePnts(Ax, x_pnts_i, Rt, K)
            R_C2W = Rt(:, 1:3);
            t_C2W = Rt(:, 4);
            
            x_pnts_i_hom = [(x_pnts_i); ones(1, size(x_pnts_i, 2))];
            x_pnts_i_cam = inv(K) * x_pnts_i_hom;


            x_pnts_i_world = R_C2W*x_pnts_i_cam + t_C2W;

            %stretch x and y coordinates for visibility
            x_pnts_i_world_exp = [1000*x_pnts_i_world(1,:); 1000*x_pnts_i_world(2,:); x_pnts_i_world(3,:)];

            scatterObj = scatter3(Ax,transpose(x_pnts_i_world_exp(1,:)), transpose(x_pnts_i_world_exp(2,:)),transpose(x_pnts_i_world_exp(3,:)), 4, "black", "filled");
        end
        %------------------------------------------------------------%


    end
end
