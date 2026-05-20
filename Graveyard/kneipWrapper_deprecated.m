function [Rt_C2W_Arr] = kneipWrapper_deprecated(x_pnts_i, X_pnts_W, K)
            %expects three points. if there are more, it will only use the
            %first three for pose calculation
           
            %Get projection rays
            x_pnts_c = createArray(3, size(x_pnts_i, 2));
            Rt_C2W_Arr = createArray(3,4,4);
        
            for j=1:size(x_pnts_i, 2)
                x_pnt_i_aug = [x_pnts_i(:,j); 1];     %Augment vector to homogenise
                K_inv = inv(K);
                
                x_pnt_c = K_inv * x_pnt_i_aug;      %Times by inverse of K. Apparently divide is faster.
                
                x_pnts_c(:,j) = x_pnt_c / norm(x_pnt_c);

            end

            %Correct for radial distortion
            %x_ABCD_c = p3pFuncs.fixRadialDistortion(x_ABCD_c, -0.3434, 0.1096);

            %Run Kneip's p3p to get up to 4 solutions for the Rt matrix.
            %(comes out as 3x16 tR matrix)
            %[Rt_C2W_Arr] = opengv('p3p_kneip', X_pnts_W(:,1:4), x_pnts_c(:,1:4));
            Rt_C2W_Arr_flat = KneipP3P_Or(X_pnts_W(:,1:3), x_pnts_c(:,1:3));
            for a=1:floor(size(Rt_C2W_Arr_flat,2)/4)
                idx = (a*4)-3;
                Rt_C2W_Arr(1:3,4,a) = Rt_C2W_Arr_flat(1:3,idx);
                Rt_C2W_Arr(1:3,1:3,a) = Rt_C2W_Arr_flat(1:3,(idx+1):(idx+3));
                %reshape(Rt_C2W_Arr_flat, 3, 4, []);
            end
end

