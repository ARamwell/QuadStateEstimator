classdef ES_EKF_3dQuad_funcs
    %Functions to support error-state EKF for 16 element state vector (including gyro
    %and accelerometer bias). 

    %nominal state vector
    % x = [position, quaternion orientation, velocity, accel bias, gyro bias]^T 
    methods (Static)

      %------------------------------------------------------------%
      function [x_new_hat]=update_nominalState(x_k, u_new, t_delta, g)
            
            xhowBig = size(x_k, 1); %programmatically set the type of EKF, 16 or 10 el
            integ = 'rect';

            if xhowBig ==10
                 w_new = zeros(6,1); %process noise assumed to be zero-mean gaussian
                 xSizeStr = '10el_';
            else
                 w_new = zeros(12,1); %process noise assumed to be zero-mean gaussian
                 xSizeStr = '16el_';
            end

            %Extract variables to match symbolic toolbox output
            numerics = [x_k; u_new; w_new; t_delta; g];

            %predefine variables for coder
            x_new_hat = createArray(xhowBig, 1);

            % ***** Evaluate model only (don't need jacobians)*****

            %Run dynamics update
            x_new_hat(1:end,1) = feval(strcat('ekf_processModel_', xSizeStr, integ), numerics); 

            % Check quaternion term
            %Enforce Smallest angle change
            if (dot(x_k(4:7), x_new_hat(4:7)) < 0)
                x_new_hat(4:7) = -x_new_hat(4:7);
            end
            %Normalise
            x_new_hat(4:7) = x_new_hat(4:7) / norm(x_new_hat(4:7)); 

      end

      %------------------------------------------------------------%
      
      function [x_new, dx_new, P_new, x_new_hat, dx_new_hat, P_new_hat, z_new_hat, z_new, y_new, K_new, S_new_hat, Q_k, W_new] = EKF_loop(g, x_k, P_k, u_new, Q_k, z_new, W_k, t_delta)
        %EKF_LOOP Main EKF loop for 3D quad, calling prediction and correction stages
        
        %*************************************************
        %----------- STEP 0: INITIALISATIONS -------------
         K_new = createArray(size(x_k, 1), 7);
         S_new_hat = nan(7,7);
         dx_new_hat = zeros(15,1); %reset error-state to zero
        
        %*************************************************
        %----------- STEP 1: DYNAMICS UPDATE -------------

            %System cannot handle omegas of perfectly 0:
            if norm(u_new(1:3)) < 0.001
                u_new(1:3) = [0.000001; 0.000001; 0.000001];
            end

            %Predict new nominal state (a priori)
            x_new_hat = update_nominalState(g, x_k, u_new, t_delta);
        
            %Predict new error state (a priori) and get jacobian 
            [dx_new_hat, F_new, L_new] = ES_EKF_3dQuad_funcs.dyn_update(g, x_k, u_new, t_delta);
            
            %Update nominal state with predicted error
            %do nothing

            %Predict new state covariance (in state space)
            P_new_hat = F_new * P_k * transpose(F_new) + L_new * Q_k * transpose(L_new);

            %Enforce positive definite-ness
            P_new_hat = (P_new_hat + P_new_hat')/2;
 
        %*************************************************
        %ONLY RUN CORRECTION IF A NEW MEASUREMENT HAS BEEN DETECTED

            if isnan(z_new(1,1))
                x_new = x_new_hat;
                dx_new = dx_new_hat;
                P_new = P_new_hat;
                z_new_hat = nan(7,1);
                y_new = nan(7,1);
                %C_new = C_k;
                W_new = W_k;
               
            else
        %---------- STEP 2: MEASUREMENT UPDATE - CAMERA ------------    

                %Predict new measurement z (may differ from actual measurement) and get
                %jacobian H, and measurement residual model
                [z_new_hat, H_new] = ES_EKF_3dQuad_funcs.meas_predict(x_new_hat);
            
                %Enforce quaternion constraints - closest quaternions
                if dot(x_k(4:7), z_new(4:7)) < 0
                    z_new(4:7) = -z_new(4:7);
                end
                if dot(z_new(4:7), z_new_hat(4:7)) < 0
                    z_new_hat(4:7) = -z_new_hat(4:7);
                end

                %Calculate measurement residual y
                y_new = z_new - z_new_hat;
            
                %Compute predicted measurement covariance S
                S_new_hat = H_new * P_new_hat * transpose(H_new) + W_k;

                %Enforce positive definite-ness
                S_new_hat = (S_new_hat + S_new_hat')/2;

        
        %*************************************************
        %------------- STEP 3: STATE UPDATE -------------- 
                
                %Calculate Kalman gain
                K_new = P_new_hat * transpose(H_new)/(S_new_hat);
            
                %Update error state est
                dx_new = dx_new_hat + (K_new * y_new);

                %Update nominal state
                x_new = x_new_hat + dx_new;
            
                %Update state covariance
                I = eye(size(H_new,2), size(H_new,2)); %make identity matrix of appropriate size
                P_new = (I - K_new * H_new) * P_new_hat;
                
                %Enforce positive definite-ness
                P_new = (P_new + P_new')/2;
       
            end

        %*************************************************
        %------------- PREPARE FINAL OUTPUT -------------- 


            %Enforce quaternion constraints - closest quaternions
            if dot(x_k(4:7), x_new(4:7)) < 0
                x_new(4:7) = -x_new(4:7);
            end

            %normalise orientation quaternion
            if norm(x_new(4:7,1)) > 0.001
                x_new(4:7,1) =x_new(4:7,1)/norm(x_new(4:7,1));
            end        
            
        end
      
      %------------------------------------------------------------%
 
      function [dx_new_hat, F_new_hat, L_new_hat] = dyn_update(dx_k, x_k, u_new, t_delta)
        %DYN_UPDATE Predict a priori error-state using dynamics and IMU

        % *****  Initialisations  *****
            
            %Extract variables to match symbolic toolbox output
            numerics = [x_k; dx_k, u_new; w_new; t_delta];

            %predefine variables for coder
            dx_new_hat = createArray(15, 1);
            F_new_hat = createArray(15,15);
            L_new_hat = createArray(15,12);

        % ***** Evaluate model and jacobians*****

            %Run dynamics update
            dx_new_hat(1:end,1) = feval(strcat('es_ekf_processModel'), numerics);
            F_new_hat(1:end, 1:end) = feval(strcat('es_ekf_F'), numerics);
            L_new_hat(1:end, 1:end) = feval(strcat('es_ekf_L'), numerics);
            
        end
           
      %------------------------------------------------------------%
      
      function [z_new_hat, H_new_hat] = meas_predict(x_new_hat)

          %predefine variables for coder
          zhowBig = 7;
          xhowBig = size(x_new_hat, 1);

          z_new_hat = createArray(zhowBig, 1);
          H_new_hat = createArray(zhowBig, xhowBig);
    
          z_new_hat(1:end, 1) = feval(strcat('es_ekf_measModel'), x_new_hat); %get predicted measurement
          H_new_hat(1:end, 1:end) = feval(strcat('es_ekf_H'), x_new_hat); %and covariance
                                     
        end

      %------------------------------------------------------------%

      % function [z_new_hat, H_new_hat] = grav_predict(x_new_hat, type)
      % 
      % 
      %     %predefine variables for coder
      %     zhowBig = 3;
      %     xhowBig = size(x_new_hat, 1);
      %     xSizeStr = strcat(string(xhowBig), 'el');
      %     z_new_hat = createArray(zhowBig, 1);
      %     H_new_hat = createArray(zhowBig, xhowBig);
      % 
      %     if type==1
      %         z_new_hat(1:end, 1) = feval(strcat('ekf_gravModel_', xSizeStr), x_new_hat); %get predicted measurement
      %         H_new_hat(1:end, 1:end) = feval(strcat('ekf_HGrav_', xSizeStr), x_new_hat); %and covariance
      %     elseif type ==2          
      %         zhowBig = 4;
      %         z_new_hat = createArray(zhowBig, 1);
      %         H_new_hat = createArray(zhowBig, xhowBig);
      %         z_new_hat(1:end, 1) = feval(strcat('ekf_gravUnitModel_', xSizeStr), x_new_hat); %get predicted measurement
      %         H_new_hat(1:end, 1:end) = feval(strcat('ekf_HUnitGrav_', xSizeStr), x_new_hat); %and covariance
      %     elseif type==3
      %         z_new_hat(1:end, 1) = feval(strcat('ekf_gravModel2_', xSizeStr), x_new_hat); %get predicted measurement
      %         H_new_hat(1:end, 1:end) = feval(strcat('ekf_HGrav2_', xSizeStr), x_new_hat); %and covariance
      %     end
      % 
      % 
      % end
      %       %------------------------------------------------------------%
      % 
      % function [z_new_hat, H_new_hat] = gravUnit_predict(x_new_hat)
      % 
      %     %predefine variables for coder
      %     zhowBig = 4;
      %     xhowBig = size(x_new_hat, 1);
      %     xSizeStr = strcat(string(xhowBig), 'el');
      %     z_new_hat = createArray(zhowBig, 1);
      %     H_new_hat = createArray(zhowBig, xhowBig);
      % 
      %     z_new_hat(1:end, 1) = feval(strcat('ekf_gravUnitModel_', xSizeStr), x_new_hat); %get predicted measurement
      %     H_new_hat(1:end, 1:end) = feval(strcat('ekf_HGravUnit_', xSizeStr), x_new_hat); %and covariance
      % 
      %   end
      % 
      % %------------------------------------------------------------%
      % 
      % function calcProcessModel(T_imu2rq)
      %   %CALCPROCESSMODEL Generates process model and jacobian functions using symbolic toolbox
      % 
      %       %general veriables
      %       R_imu2rq = T_imu2rq(1:3,1:3);
      % 
      % 
      %       %Define symbolic variables
      % 
      %       %general
      %       dt = sym("dt");
      % 
      %       %nominal state - a constant, essentially
      %       p = sym("p", [3,1], 'real');
      %       q = sum("q", [4,1], 'real');
      %       v =sym("v", [3,1],  'real');
      %       %R_rq2rw = sym("R", [3,3], 'real'); %rotation matrix describing current best estimate for robot orientation in world frame
      %       ba = sym("ba", [3,1], 'real'); %current best estimate for accelerometer bias
      %       bg = sym("bg", [3,1], 'real'); %current best estimate for gyroscope bias
      %       x = [p; q; v; ba; bg]; %nominal state vector
      % 
      %       %old error state
      %       dp = sym("dp", [3,1], 'real');
      %       dv = sym("dv", [3,1],  'real');
      %       do = sym("do", [3,1],  'real'); %angular error 
      %       dba = sym("dba", [3,1],  'real');
      %       dbg = sym("dbg", [3,1], 'real');
      %       dx = [dp; do; dv; dba; dbg]; %error state vector
      % 
      %       %new "control input"
      %       u_g_new = sym("u_g_new", [3,1], 'real');
      %       u_a_new = sym("u_a_new", [3,1], 'real');
      %       u_new = [u_g_new; u_a_new];
      % 
      %       %process noise
      %       w_g = sym("w_g", [3,1], 'real');
      %       w_a = sym("w_a", [3,1], 'real');
      %       w_bg = sym("w_bg", [3,1], 'real');
      %       w_ba = sym("w_ba", [3,1], 'real');
      %       w = [w_g; w_a; w_ba; w_bg];
      % 
      %       %prev "control input"
      %       u_g_old = sym("u_g_prev", [3,1], 'real');
      %       u_a_old = sym("u_a_prev", [3,1], 'real');
      %       u_old = [u_g_old; u_a_old];
      % 
      %       %compile symbols
      %       symbols = [x; dx; u_new; w; dt];
      % 
      %       %convert to some useful forms
      %       R_rq2rw= [1 - 2*(q(3)^2 + q(4)^2), 2*(q(2)*q(3) - q(4)*q(1)), 2*(q(2)*q(4) + q(3)*q(1)); % Convert q to rotation matrix
      %                 2*(q(2)*q(3) + q(4)*q(1)), 1 - 2*(q(2)^2 + q(4)^2), 2*(q(3)*q(4) - q(2)*q(1));
      %                 2*(q(2)*q(4) - q(3)*q(1)), 2*(q(3)*q(4) + q(2)*q(1)), 1 - 2*(q(2)^2 + q(3)^2)];
      % 
      % 
      %       %***** define dynamic equations *****
      %       %using rectangular integration where applicable
      %       dba_new = dba + w_ba;
      %       dbg_new = dbg + w_bg;
      % 
      %       do_new = do + R_rq2rw * R_imu2rq * (dbg_new + w_bg) * dt;
      % 
      %       angErr = R_rq2rw * R_imu2rq * (u_a_new - ba - w_a - w_ba); %temp
      %       angErr = [0        , -angErr(1), -angErr(2);
      %                 angErr(1), 0         , -angErr(3);
      %                 angErr(2), angErr(3) , 0         ]; %skew symmetric form
      %       dv_new = dv + ((-angErr * do_new) - (R_rq2rw * R_imu2rq * dba))*dt;
      % 
      %       dp_new = dp + dv_new*dt;
      % 
      %       %***** generate complete process model *****
      %       dx_new = [dp_new; do_new; dv_new; dba_new; dbg_new]; 
      % 
      %       % define jacobians                      
      %       F_star = jacobian(dx_new, dx); 
      %       L_star = jacobian(dx_new, w);
      % 
      %       % generate and save functions
      %       matlabFunction(F_star, 'File', strcat('Functions/mainEstimator/es-ekf/es_ekf_F'), 'Vars', {symbols}, 'Optimize', true);
      %       matlabFunction(L_star, 'File', strcat('Functions/mainEstimator/es-ekf/es_ekf_L'), 'Vars', {symbols}, 'Optimize', true);
      %       matlabFunction(dx_new, 'File', strcat('Functions/mainEstimator/es-ekf/es_ekf_processModel'), 'Vars', {symbols}, 'Optimize', true);
      % 
      %   end

      %------------------------------------------------------------%

      function calcMeasurementModel(T_rc2rq)
            %Function using symbolic toolbox to calculate the matrices involved in
            %the CAMERA measurement model, for the error state EKF.
           
            %Extract useful variables
            R_rc2rq = (T_rc2rq(1:3, 1:3));
            t_rc2rq = T_rc2rq(1:3, 4);
            q_rc2rq = rotm2quat(R_rc2rq)';
  
            %Define symbolic variables

            %nominal state - a constant, essentially
            p = sym("p", [3,1], 'real');
            q = sum("q", [4,1], 'real');
            v =sym("v", [3,1],  'real');
            %R_rq2rw = sym("R", [3,3], 'real'); %rotation matrix describing current best estimate for robot orientation in world frame
            ba = sym("ba", [3,1], 'real'); %current best estimate for accelerometer bias
            bg = sym("bg", [3,1], 'real'); %current best estimate for gyroscope bias
            x = [p; q; v; ba; bg]; %nominal state vector
          
            %old error state
            dp = sym("dp", [3,1], 'real');
            dv = sym("dv", [3,1],  'real');
            do = sym("do", [3,1],  'real'); %angular error 
            dba = sym("dba", [3,1],  'real');
            dbg = sym("dbg", [3,1], 'real');
            dx = [dp; do; dv; dba; dbg]; %error state vector

            %new "control input"
            u_g_new = sym("u_g_new", [3,1], 'real');
            u_a_new = sym("u_a_new", [3,1], 'real');
            u_new = [u_g_new; u_a_new];

            %process noise
            w_g = sym("w_g", [3,1], 'real');
            w_a = sym("w_a", [3,1], 'real');
            w_bg = sym("w_bg", [3,1], 'real');
            w_ba = sym("w_ba", [3,1], 'real');
            w = [w_g; w_a; w_ba; w_bg];

            %prev "control input"
            u_g_old = sym("u_g_prev", [3,1], 'real');
            u_a_old = sym("u_a_prev", [3,1], 'real');
            u_old = [u_g_old; u_a_old];

            %compile symbols

            symbols = [x; dx; u_new; w; dt];
           
            %Extract rotation matrix
            R_rq2rw =  [1 - 2*(q(3)^2 + q(4)^2), 2*(q(2)*q(3) - q(4)*q(1)), 2*(q(2)*q(4) + q(3)*q(1)); % Convert q to rotation matrix
                2*(q(2)*q(3) + q(4)*q(1)), 1 - 2*(q(2)^2 + q(4)^2), 2*(q(3)*q(4) - q(2)*q(1));
                2*(q(2)*q(4) - q(3)*q(1)), 2*(q(3)*q(4) + q(2)*q(1)), 1 - 2*(q(2)^2 + q(3)^2)];
                      
            lmo_rq2rw = [q(1) -q(2) -q(3) -q(4) %turn q into left matrix operator for easier math
                        q(2) q(1) -q(4) q(3)
                        q(3) q(4) q(1) -q(2)
                        q(4) -q(3) q(2) q(1)];      

            %Manual quaternion rotation: q_rq2rw*q_rc2rq
            q_rc2rw = lmo_rq2rw * q_rc2rq;
            
            %apply camera measurement model - if we get pose of camera in
            %world
            p_z = R_rq2rw * t_rc2rq + p;
            q_z = q_rc2rw;

            %and if we instead get pose of quad in world?
            p_z = p;
            q_z = q;

            % OUTPUTS
            measModel = [p_z; q_z]; %predicted measurement vector (symbolic)
           
            % define jacobians
            H_x = jacobian(measModel, x); %jacobian of measurement wrt to nominal state
            X_dx = eye(16,15);
            X_dx(4:7, 4:6) = [-q(2), -q(3), -q(4);
                               q(1), -q(4),  q(3);
                               q(4),  q(1), -q(2);
                              -q(3),  q(2),  q(1)] * 0.5;
            H_star = H_x * X_dx;
            %H_star = jacobian(measModel, dx); %jacobian of measurement wrt state

            % generate and save functions
            matlabFunction(measModel, 'File', strcat('Functions/mainEstimator/es-ekf/es_ekf_measModel'), 'Vars', {symbols}, 'Optimize', true);
            matlabFunction(H_star, 'File', strcat('Functions/mainEstimator/es-ekf/es_ekf_H'), 'Vars', {symbols}, 'Optimize', true);
      end

      %------------------------------------------------------------%

        % function calcGravMeasurementModel(g)
        %     %Function using symbolic toolbox to calculate the matrices involved in
        %     %the GRAVITY VECTOR measurement model. Here, the raw
        %     %accelerometer reading is the measurement.
        % 
        %     %Define symbolic variables
        % 
        %     %state
        %     p = sym("p", [3,1]);
        %     v = sym("v", [3,1]);
        %     q = sym("q", [4,1]);
        %     ba = sym("ba", [3,1]);
        %     bg = sym("bg", [3,1]);
        %     x_16el = [p; q; v; ba; bg];
        %     x_10el = [p; q; v];
        % 
        %     %compile used symbols
        %     symbols_16 = x_16el; %state only
        %     symbols_10 = x_10el;
        % 
        %     % %sanity check on euler states
        %     % yaw = sym("yaw", [1,1]);
        %     % pitch = sym("pit", [1,1]);
        %     % roll = sym("roll", [1,1]);
        %     % cy = cos(yaw * 0.5);
        %     % sy = sin(yaw * 0.5);
        %     % cp = cos(pitch * 0.5);
        %     % sp = sin(pitch * 0.5);
        %     % cr = cos(roll * 0.5);
        %     % sr = sin(roll * 0.5);
        %     % 
        %     % % Calculate quaternion components [w, x, y, z]
        %     % w = cy * cp * cr + sy * sp * sr;
        %     % x = cy * cp * sr - sy * sp * cr;
        %     % y = sy * cp * sr + cy * sp * cr;
        %     % z = sy * cp * cr - cy * sp * sr;
        %     % 
        %     % q = [w, x, y, z];
        % 
        % 
        %     %Extract rotation matrix
        %     R_rq2rw =  [1 - 2*(q(3)^2 + q(4)^2), 2*(q(2)*q(3) - q(4)*q(1)), 2*(q(2)*q(4) + q(3)*q(1)); % Convert q to rotation matrix
        %         2*(q(2)*q(3) + q(4)*q(1)), 1 - 2*(q(2)^2 + q(4)^2), 2*(q(3)*q(4) - q(2)*q(1));
        %         2*(q(2)*q(4) - q(3)*q(1)), 2*(q(3)*q(4) + q(2)*q(1)), 1 - 2*(q(2)^2 + q(3)^2)];
        % 
        %     %Meas model: 
        %     a_z = (R_rq2rw)'*(-g)+ba; %For the 10-element EKF, we could use the normalised acceleration and gravity vector, since bias is not observed anyway. 
        %     a_z_alt = (R_rq2rw)'*(-g);
        %     %a_z_unit = (R_rq2rw)'*(-g/vecnorm(g)); %Try unit version - first try
        %     a_z_unit = a_z/sqrt(a_z(1)^2+a_z(2)^2+a_z(3)^2);
        %     len = sqrt(a_z(1)^2+a_z(2)^2+a_z(3)^2);
        % 
        % 
        %     % OUTPUTS - case 1
        %     gravModel_16el = [a_z]; %predicted measurement vector (symbolic)
        %     gravModel_10el = subs(gravModel_16el, [ba; bg], zeros(2*3, 1)); %simplify by evaluating with biases and associated noise = 0
        % 
        %     % define jacobians
        %     HGrav_star_10el = jacobian(gravModel_10el, x_10el); %jacobian of measurement wrt state
        %     HGrav_star_16el = jacobian(gravModel_16el, x_16el); %jacobian of measurement wrt state
        % 
        %     % generate and save functions
        %     matlabFunction(gravModel_10el, 'File', strcat('Functions/mainEstimator/ekf/ekf_gravModel_10el'), 'Vars', {symbols_10}, 'Optimize', true);
        %     matlabFunction(gravModel_16el, 'File', strcat('Functions/mainEstimator/ekf/ekf_gravModel_16el'), 'Vars', {symbols_16}, 'Optimize', true);
        %     matlabFunction(HGrav_star_10el, 'File', strcat('Functions/mainEstimator/ekf/ekf_HGrav_10el'), 'Vars', {symbols_10}, 'Optimize', true);
        %     matlabFunction(HGrav_star_16el, 'File', strcat('Functions/mainEstimator/ekf/ekf_HGrav_16el'), 'Vars', {symbols_16}, 'Optimize', true);
        % 
        %     % OUTPUTS - case 3 (no bias!)
        %     gravModel2_16el = [a_z_alt]; %predicted measurement vector (symbolic)
        %     gravModel2_10el = subs(gravModel2_16el, [ba; bg], zeros(2*3, 1)); %simplify by evaluating with biases and associated noise = 0
        % 
        %     % define jacobians
        %     HGrav2_star_10el = jacobian(gravModel2_10el, x_10el); %jacobian of measurement wrt state
        %     HGrav2_star_16el = jacobian(gravModel2_16el, x_16el); %jacobian of measurement wrt state
        % 
        %     % generate and save functions
        %     matlabFunction(gravModel2_10el, 'File', strcat('Functions/mainEstimator/ekf/ekf_gravModel2_10el'), 'Vars', {symbols_10}, 'Optimize', true);
        %     matlabFunction(gravModel2_16el, 'File', strcat('Functions/mainEstimator/ekf/ekf_gravModel2_16el'), 'Vars', {symbols_16}, 'Optimize', true);
        %     matlabFunction(HGrav2_star_10el, 'File', strcat('Functions/mainEstimator/ekf/ekf_HGrav2_10el'), 'Vars', {symbols_10}, 'Optimize', true);
        %     matlabFunction(HGrav2_star_16el, 'File', strcat('Functions/mainEstimator/ekf/ekf_HGrav2_16el'), 'Vars', {symbols_16}, 'Optimize', true);
        % 
        % 
        %     % OUTPUTS - unit vector case
        %     gravUnitModel_16el = [a_z_unit; len]; %predicted measurement vector (symbolic)
        %     gravUnitModel_10el = subs(gravUnitModel_16el, [ba; bg], zeros(2*3, 1)); %simplify by evaluating with biases and associated noise = 0
        % 
        %     % define jacobians
        %     HGravUnit_star_10el = jacobian(gravUnitModel_10el, x_10el); %jacobian of measurement wrt state
        %     HGravUnit_star_16el = jacobian(gravUnitModel_16el, x_16el); %jacobian of measurement wrt state
        % 
        %     % generate and save functions
        %     matlabFunction(gravUnitModel_10el, 'File', strcat('Functions/mainEstimator/ekf/ekf_gravUnitModel_10el'), 'Vars', {symbols_10}, 'Optimize', true);
        %     matlabFunction(gravUnitModel_16el, 'File', strcat('Functions/mainEstimator/ekf/ekf_gravUnitModel_16el'), 'Vars', {symbols_16}, 'Optimize', true);
        %     matlabFunction(HGravUnit_star_10el, 'File', strcat('Functions/mainEstimator/ekf/ekf_HGravUnit_10el'), 'Vars', {symbols_10}, 'Optimize', true);
        %     matlabFunction(HGravUnit_star_16el, 'File', strcat('Functions/mainEstimator/ekf/ekf_HGravUnit_16el'), 'Vars', {symbols_16}, 'Optimize', true);
        % 
        % end
  
      %------------------------------------------------------------%

        function [P_0, Q, W] = initEKF_params(dt_av, x_0, ekfHz)

            xSize = size(x_0, 1);

            %Define P_k - Initial State covariance
             
            %P_robmech
            % P_ = diag([0.001, 0.001, 0.001, ...
            % 0.001, 0.001, 0.001, 0.001, ...
            % 0.001, 0.001, 0.001, ...
            % 0.2, 0.2, 0.2, ...
            % 0.01, 0.01, 0.01]); %Initial, 16 el

            %P_px4 - position, orientation, velocity error is a bit made up
            P_ = diag([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.02, 0.02, 0.02, 0.01, 0.01, 0.01]); %Initial, 16 el, good bias calib

            P_ = diag([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.15, 0.15, 0.15, 0.05, 0.05, 0.05]); %Initial, 16 el, bad bias calib

            %P_ = diag([0.2, 0.2, 0.2, 0.2*0.01, 0.2*0.1, 0.2*0.1, 0.2*0.1, 0.2, 0.2, 0.2, 1, 1, 1, 0.05, 0.05, 0.05]); %Initial, 16 el, REALLY  bad bias calib
            
            %
            if xSize == 10
                P_0 = P_(1:10, 1:10);
            else
                P_0 = P_;
            end

            % if ekfHz <=2000
            %     ka = (ekfHz/250)^2;
            %     kg = ka;
            % else %the downsampling makes noise scale weirdly
            %     ka = (ekfHz/250)^2;
            %     kg = (2000/250)^2;
            % end
            % %kg = kg*30*1000; %old multiplier
            % %ka=ka*30*100;
            % w_g = kg*[0.058^2 0.062^2 0.080^2]';%*8000/2000; %1 SD @ 8Khz
            % w_a = ka*[0.031^2 0.028^2 0.031^2]'; %1 SD @ 8kHz

            %process noise covariance (noise space)            
            %t_delta_dur = ((u_timeHist(1,2) - u_timeHist(1,1)));
            %t_delta = milliseconds(t_delta_dur) * 0.001;
            % w_g = [(1.037e-03)^2, 0.001^2, 0.00129^2]' / dt_av; %from Allan variance
            % w_g = [0.0029^2, 0.005^2, 0.0037^2]';
            % w_a = [(5.809e-03)^2, 0.008^2, 0.011^2]' / dt_av; %from Allan variance
            % w_a = [0.0128^2, 0.0164^2, 0.0191^2 ]';
            % w_ba = [(1.081e-05)^2, (0.000001)^2, (6.55e-06)^2]' /dt_av; %from Allan variance
            % w_bg = [(1e-07)^2, (1e-07)^2, (1e-07)^2]' /dt_av; %from Allan variance
            
            

            % from full rate histogram
            %w_a = [0.02^2 0.02^2 0.02^2];%used for full rate
            %w_g = [0.04^2 0.04^2 0.04^2];

            % from batch-integrated histogram @250Hz
            %w_a = [0.005^2 0.005^2 0.005^2]';
            %w_g = [0.02^2 0.02^2 0.02^2]';


            %w_ba = 0.01*w_a;
            %w_bg = 0.01*w_g;

            %w_g = [0.015^2 0.015^2 0.015^2]';% PX4
            %w_a = [0.35^2 0.35^2 0.35^2]'; %PX4
            %w_ba = [0.01^2 0.01^2 0.01^2]';
            %w_bg = [0.001^2 0.001^2 0.001^2]';

            % %%TUNE 1
            % w_a = [0.02^2 0.02^2 0.02^2];%used for full rate
            % w_g = [0.04^2 0.04^2 0.04^2];
            % w_ba = 0.01*w_a;
            % w_bg = 0.01*w_g;

            %%TUNE 2
            % w_a = 10*[0.02^2 0.02^2 0.02^2];%used for full rate
            % w_g = 10*[0.04^2 0.04^2 0.04^2];
            % w_ba = [0.02^2 0.02^2 0.02^2];
            % w_bg = 0.01*[0.04^2 0.04^2 0.04^2];

            % %%TUNE 3
            % w_a = 50^2*[0.02^2 0.02^2 0.02^2];%used for full rate
            % w_g = 50^2*[0.04^2 0.04^2 0.04^2];
            % w_ba = 50^2*0.01*[0.02^2 0.02^2 0.02^2];
            % w_bg = 0.01*[0.04^2 0.04^2 0.04^2];
            
            % %%TUNE 4
            % w_a = 100*[0.02^2 0.02^2 0.02^2];%used for full rate
            % w_g = 10*[0.04^2 0.04^2 0.04^2];
            % w_ba = [0.2^2 0.2^2 0.2^2];
            % w_bg = 0.01*[0.04^2 0.04^2 0.04^2];

            %%TUNE 5
            % w_a = [0.3^2 0.3^2 0.3^2];%used for full rate
            % w_g = 10*[0.04^2 0.04^2 0.04^2];
            % w_ba = [0.4^2 0.4^2 0.4^2];
            % w_bg = 0.01*[0.04^2 0.04^2 0.04^2];

            %%TUNE 1S
            w_a = [0.02^2 0.02^2 0.02^2];%used for full rate
            w_g = [0.08^2 0.08^2 0.08^2];
            w_ba = 0.01*w_a;
            w_bg = 0.01*w_g;

            %%Testing gravity aiding... does bias process noise have to be
            %%mega high initially?
            w_ba = w_ba *1000;
            %w_bg = w_bg*1000;

            %%TUNE 2S
             % w_a = [0.04^2 0.04^2 0.04^2];%used for full rate
             % w_g = [0.16^2 0.16^2 0.16^2];
             % w_ba = [0.008^2 0.008^2 0.008^2];
             % w_bg = [0.008^2 0.008^2 0.008^2];

            % %%TUNE 3S
              % w_a = [0.1^2 0.1^2 0.1^2];%used for full rate
              % w_g = [0.2^2 0.2^2 0.2^2];
              % w_ba = [0.02^2 0.02^2 0.02^2];
              % w_bg = [0.008^2 0.008^2 0.008^2];

             %  %%TUNE 4S
             %  w_a = [0.15^2 0.15^2 0.15^2];%used for full rate
             %  w_g = [0.2^2 0.2^2 0.2^2];
             %  w_ba = [0.05^2 0.05^2 0.05^2];
             %  w_bg = [0.008^2 0.008^2 0.008^2];
             % 
             %  %%TUNE 5S
             % w_a = [0.2^2 0.2^2 0.2^2];%used for full rate
             %  w_g = [0.2^2 0.2^2 0.2^2];
             %  w_ba = [0.1^2 0.1^2 0.1^2];
             %  w_bg = [0.008^2 0.008^2 0.008^2];
           
             %%TUNE 6S
             % w_a = [0.05^2 0.05^2 0.05^2];%used for full rate
             % w_g = [0.1^2 0.1^2 0.1^2];
             % w_ba = 0.1*w_a;
             % w_bg = 0.01*w_g;
             % 
             % %%TUNE 7S
             % w_a = [0.02^2 0.02^2 0.02^2];%used for full rate
             % w_g = [0.08^2 0.08^2 0.08^2];
             % w_ba = w_a;
             % w_bg = 0.01*w_g;



           
            Q_ =diag([w_g w_a w_ba w_bg]);

            

            %Q_ Robmech
           % Q_ = diag([0.01, 0.01, 0.01, 0.2, 0.2, 0.2, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001]);%*(16/1000); %16 el - [w_g, w_acc, w_ba, w_ba]
            
           %Q_Cust
           % Q_ = 10* diag([1.5e-1, 1.5e-1, 1.5e-1, 2e-1, 2e-1, 2e-1, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3]); %16 el - [w_g, w_acc, w_ba, w_ba] % cust 

            %Q_Modified
            %Q_ = 10* diag([1.5e-1, 1.5e-1, 1.5e-1, 2e-2, 2e-2, 2e-2, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3]); %16 el - [w_g, w_acc, w_ba, w_ba] 

           %Q_Modified 2
           % Q_ = diag([1.5e-1, 1.5e-1, 1.5e-1, 2e-1, 2e-1, 2e-1, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3]); %16 el - [w_g, w_acc, w_ba, w_ba] % cust 

            %Q_PX4
            % w_g = 0.015;
            % w_a = 0.35;
            % w_ba= 0.003;
            % w_bg = 0.001;
            % Q_ = diag([w_g^2 w_g^2 w_g^2 w_a^2 w_a^2 w_a^2 w_ba^2 w_ba^2 w_ba^2 w_bg^2 w_bg^2 w_bg^2]); %16 el - [w_g, w_acc, w_ba, w_ba]

            
            if xSize == 10
                Q = Q_(1:6, 1:6);
            else
                Q = Q_;
            end
            Q = Q ;%*(8000/ekfHz);

            %ACTUALLY, accidentally ran EKFs with my W but px4's Q! (for
            %the "modified custom". Let's run again with my true mod.

            %and measurement covariance
            %W =diag([0.1506^2, 0.1506^2, 0.1506^2, 0.01, 0.007897, 0.007897, 0.007897]);  %robmech
            
            %angle_err = 0.37+0.34;%low distortion lens nano, mean + sd
            %pos_err = 0.08+0.08;%low distortion lens nano, mean + sd
            
            %angle_err =sqrt(5.73);%Px4
            %pos_err = sqrt(0.2);%PX4
            
            %q_err = (deg2rad(angle_err))^2/4;
            %W =diag([pos_err^2, pos_err^2, pos_err^2, 0.1, q_err, q_err, q_err]);  
            
            W = diag([0.13^2 0.13^2 0.13^2 0.004^2 0.045^2 0.045^2 0.045^2]); %from p3p histograms
            
            %W = diag([0.038^2 0.038^2 0.038^2 0.025^2 0.028^2 0.028^2 0.028^2]); %means from adaptive 10elR at 250Hz ******** - cust settings all cases
            

        end
        %------------------------------------------------------------%


        %------------------------------------------------------------%

    
    end
    
end