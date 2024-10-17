classdef p3pRun
    methods (Static)
        
        function [Rt_CW_Grun, Err_Grun, Rt_CW_arr, Rt_WC_arr] = Grunert(x_pnts_i, X_pnts_W, K, checkerSize,inlierThreshold)

           
            %Get outside corners
            if size(X_pnts_W, 2) <=4
                x_ABCD_i = x_pnts_i;
                X_ABCD_W = X_pnts_W;
            else
                [x_ABCD_i, X_ABCD_W] = p3pFuncs.checkerOuterCornerSelector(x_pnts_i, X_pnts_W, checkerSize(1), checkerSize(2));
            end
            %Extract verification pnt
            x_D_i = x_ABCD_i(:,4);
            X_D_W = X_ABCD_W(:,4);
            
            %Get projection rays
            x_ABCD_c = p3pFuncs.getCameraVector(K, x_ABCD_i);

            %Run Grunert's p3p to get up to 4 solutions for the Rt matrix
            Rt_CW_arr = GrunertP3P(X_ABCD_W(:,1:3), x_ABCD_c(:,1:3));

            %Reprojection error is calculated based on the W->i projection,
            %therefore the above matrix must be inverted
            Rt_WC_arr = Rt_CW_arr;
            for j=1:size(Rt_CW_arr,3)
                Rt_WC_arr(:,:,j) = p3pFuncs.invertRt(Rt_CW_arr(:,:,j));
            end 

            [Rt_WC, Err] = p3pFuncs.chooseRtWithMinReprojErrorWC(K, Rt_WC_arr, x_D_i, X_D_W);
            
            Rt_CW_Grun = p3pFuncs.invertRt(Rt_WC);

        end

        
        function [Rt, Err, Rt_CW_arr, Rt_WC_arr] = KneipA(x_pnts_i, X_pnts_W, K, checkerSize,inlierThreshold)

            %Get outside corners
            if size(X_pnts_W, 2) <=4
                x_ABCD_i = x_pnts_i;
                X_ABCD_W = X_pnts_W;
            else
                [x_ABCD_i, X_ABCD_W] = p3pFuncs.checkerOuterCornerSelector(x_pnts_i, X_pnts_W, checkerSize(1), checkerSize(2));
            end
            x_D_i = x_ABCD_i(:,4);
            X_D_W = X_ABCD_W(:,4);
            
            
            %Get projection rays
            x_ABCD_c = p3pFuncs.getCameraVector(K, x_ABCD_i);



            %Run Alyssa's implementation of Kneip's p3p to get up to 4 solutions for the Rt matrix
            Rt_WC_arr = KneipP3P_Al(X_ABCD_W(:,1:3), x_ABCD_c(:,1:3));

            %Reprojection error is calculated based on the W->i projection,
            %therefore the above matrix must be inverted
            % Rt_WC_arr = Rt_CW_arr;
            % for j=1:size(Rt_CW_arr,3)
            %     Rt_WC_arr(:,:,j) = p3pFuncs.invertRt(Rt_CW_arr(:,:,j));
            % end 
             Rt_CW_arr = Rt_WC_arr;
             for j=1:size(Rt_CW_arr,3)
                 Rt_CW_arr(:,:,j) = p3pFuncs.invertRt(Rt_WC_arr(:,:,j));
             end 

            [Rt_CW, Err] = p3pFuncs.chooseRtWithMinReprojErrorCW(K, Rt_CW_arr, x_D_i, X_D_W);
            [Rt_WC, Err] = p3pFuncs.chooseRtWithMinReprojErrorWC(K, Rt_WC_arr, x_D_i, X_D_W);
            
            %[Rt_WC, Err] = p3pFuncs.chooseRtWithMostInliersWC(K, Rt_WC_arr, inlierThreshold, x_pnts_i, X_pnts_W);
            %[Rt_CW, Err] = p3pFuncs.chooseRtWithMostInliersCW(K, Rt_CW_arr, inlierThreshold, x_pnts_i, X_pnts_W);

            Rt_CW_der = p3pFuncs.invertRt(Rt_WC);
            Rt_WC_der = p3pFuncs.invertRt(Rt_CW);
            
            Rt =Rt_CW_der;
        end


        function [Rt, Err, Rt_CW_Arr, Rt_WC_Arr] = KneipN(x_pnts_i, X_pnts_W, K, checkerSize,inlierThreshold)

            %Initialise variables
            Rt_CW_Arr = zeros(3,4,1);
            Rt_WC_Arr = zeros(3,4,1);
            
            if size(X_pnts_W, 2) <=4
                x_ABCD_i = x_pnts_i;
                X_ABCD_W = X_pnts_W;
            else
                [x_ABCD_i, X_ABCD_W] = p3pFuncs.checkerOuterCornerSelector(x_pnts_i, X_pnts_W, checkerSize(1), checkerSize(2));
            end
            x_D_i = x_ABCD_i(:,4);
            X_D_W = X_ABCD_W(:,4);
            
            
            %Get projection rays
            x_ABCD_c = p3pFuncs.getCameraVector(K, x_ABCD_i);

            %Run Kneip's p3p to get up to 4 solutions for the Rt matrix.
            %Nagano implementation outputs W->C
            [R_WC_Arr, t_WC_Arr] = KneipP3P_Nag(x_ABCD_c(:,1:3), X_ABCD_W(:,1:3));

            Rt_CW_Arr = zeros(3,4,size(R_WC_Arr,3));

            %For each possible solution
            for j=1:size(R_WC_Arr,3)
                %Concatenate to get Rt
                Rt_WC_Arr(:,1:3,j) = R_WC_Arr(:,:,j);
                Rt_WC_Arr(:,4,j) = t_WC_Arr(:,j);

                %And invert
                Rt_CW_Arr(:,:,j) = p3pFuncs.invertRt(Rt_WC_Arr(:,:,j));
            end

            %Reprojection error is calculated based on the W->i projection,
            %therefore the above matrix must be inverted
            % Rt_WC_arr = zeros(3,4,size(Rt_CW_Arr,3));
            % for j=1:size(Rt_CW_arr,3)
            %     Rt_WC_arr(:,:,j) = p3pFuncs.invertRt(Rt_CW_arr(:,:,j));
            % end 

            %[Rt_WC, Err] = p3pFuncs.chooseRtWithMinReprojErrorWC(K, Rt_WC_Arr, x_D_i, X_D_W);
            %[Rt_CW, Err] = p3pFuncs.chooseRtWithMinReprojErrorCW(K, Rt_CW_Arr, x_D_i, X_D_W);
            
            [Rt_WC, Err] = p3pFuncs.chooseRtWithMostInliersWC(K, Rt_WC_Arr, inlierThreshold, x_pnts_i, X_pnts_W);
            [Rt_CW, Err] = p3pFuncs.chooseRtWithMostInliersCW(K, Rt_CW_Arr, inlierThreshold, x_pnts_i, X_pnts_W);
            
            
            Rt_CW_der = p3pFuncs.invertRt(Rt_WC);
            Rt_WC_der = p3pFuncs.invertRt(Rt_CW);

            Rt = Rt_CW_der;
           
        end



        function Rt_CW_Mat = MatlabPnP(K, x_pnts_i, X_pnts_W, imageSize, maxErr)

            %Extract necessary variables
            focalLength = [K(1,1) K(2,2)];
            CP = [K(1,3) K(2,3)]; %Principal point
            intr = cameraIntrinsics(focalLength, CP, imageSize);

            x_pnts_i_row = transpose(x_pnts_i);
            X_pnts_W_row = transpose(X_pnts_W);

            %Run Matlab PnP
            worldPose = estworldpose(x_pnts_i_row, X_pnts_W_row, intr, "MaxReprojectionError", maxErr);
            R=worldPose.R;
            t=transpose(worldPose.Translation);

            Rt_CW_Mat = [R t];

        end


        function Rt_CW_KneipO = KneipO(x_pnts_i, X_pnts_W, K, checkerSize, inlierThreshold)

            %Initialise Rt
            Rt_CW_arr= zeros(3,4,1);

            %Get outside corners
            if size(X_pnts_W, 2) <=4
                x_ABCD_i = x_pnts_i;
                X_ABCD_W = X_pnts_W;
            else
                [x_ABCD_i, X_ABCD_W] = p3pFuncs.checkerOuterCornerSelector(x_pnts_i, X_pnts_W, checkerSize(1), checkerSize(2));
            end
            %Extract verification pnt
            x_D_i = x_ABCD_i(:,4);
            X_D_W = X_ABCD_W(:,4);
            
            
            %Get projection rays
            x_ABCD_c = p3pFuncs.getCameraVector(K, x_ABCD_i);

            %Run Kneip's p3p to get up to 4 solutions for the Rt matrix -
            %gives 3x16 matrix of format [t R t R t R t R].
            Rc_matrix_CW = KneipP3P_Or(X_ABCD_W(:,1:3), x_ABCD_c(:,1:3));

            %Separate into 3x4x4 Rt matrix
            for i=1:4
                
                R = Rc_matrix_CW(:, (i*4 -2):(i*4));
                t = Rc_matrix_CW(:,(i*4 -3));
                Rt_CW_arr(:,1:3,i) =R;
                Rt_CW_arr(:,4,i) =t;
            end
                

            %Reprojection error is calculated based on the W->i projection,
            %therefore the above matrix must be inverted
            Rt_WC_arr = Rt_CW_arr;
            for j=1:size(Rt_CW_arr,3)
                Rt_WC_arr(:,:,j) = p3pFuncs.invertRt(Rt_CW_arr(:,:,j));
            end 

            %[Rt_WC, Err] = p3pFuncs.chooseRtWithMinReprojErrorWC(K, Rt_WC_arr, x_D_i, X_D_W);
            [Rt_WC, Err] = p3pFuncs.chooseRtWithMostInliersWC(K, Rt_WC_arr, inlierThreshold, x_pnts_i, X_pnts_W);
            
            Rt_CW_KneipO = p3pFuncs.invertRt(Rt_WC);

        end
    end
end

            
      







            