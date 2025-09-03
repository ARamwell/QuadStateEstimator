classdef EKF_3dQuad_funcs
    %Functions to support EKF for 16 element state vector (including gyro
    %and accelerometer bias). 
    methods (Static)

      %------------------------------------------------------------%
      
      function [x_new, P_new, x_new_hat, z_new_hat, z_new] = EKF_loop(g, x_k, P_k, u_new, Q, z_new, W, t_delta, T_imu2rq, T_rc2rq, integ)
        %EKF_LOOP Main EKF loop for 3D quad, calling prediction and correction stages
        %
        %   Extended Kalman filter using IMU measurements for state prediction and fusing in pose 'measurements' from some other state sensor (usually visual) 
        %
        %
        %   Inputs:
        %       g - gravity vector (column) in NED ref frame 
        %       x_k - most recent (k) a posteriori state estimate, col vector (world NED frame): [position; orientation quaternion; velocity; accel bias; gyro bias]
        %       P_k - most recent a posteriori state covariance
        %       u_k - most recent IMU measurement, col vector, [gyro; accel]
        %       Q - IMU measurement covariance, in IMU space
        %       z_k - most recent pose measurement from visual system, column vector: [position; orientation quat] 
        %       W - aiding sensor measurement covariance, sensor space
        %       t_delta - time since last EKF loop, in seconds
        %       T_imu2rq - homog transformation matrix from IMU to body frame (rq - "real quad")
        %       T_rc2rq - homog. transformation matrix from aiding sensor to body frame ("real camera" to rq)
        %       reset - flag for first run ("1"); calls jacobian matrix initialisation from current process model  
        %       integ - type of integration to use: 'rect' or 'trap'
        %   Outputs:
        %       x_new - a posteriori state estimate for k+1
        %       P_new - a posteriori state est covariance for k+1
        %       processTerm_k - state innovation term
        %       x_new_hat - a priori state estimate for k+1
        %       z_new_hat - predicted measurement for k+1
        %       z_k - output aiding state measurement - quaternion may have changed sign
        
        
        %*************************************************
        %----------- STEP 0: INITIALISATIONS -------------

       
        
        %*************************************************
        %----------- STEP 1: DYNAMICS UPDATE -------------
        
            %Predict new state (a priori) and get prev jacobian 
            [x_new_hat, F_new, L_new] = EKF_3dQuad_funcs.dyn_update(g, x_k, u_new, t_delta, T_imu2rq, integ);
            
            %enforce quaternion continuity
            if dot(x_new_hat(4:7), x_k(4:7)) < 0
                x_new_hat(4:7) = -x_new_hat(4:7);
            end
                       
            %Predict new state covariance (in state space)
            P_new_hat = F_new * P_k * transpose(F_new) + L_new * Q * transpose(L_new);
 
        %*************************************************
        %ONLY RUN CORRECTION IF A NEW MEASUREMENT HAS BEEN DETECTED

            if isnan(z_new)
                x_new = x_new_hat;
                P_new = P_new_hat;
                z_new_hat = zeros(7,1);
            
            else
        %---------- STEP 2: MEASUREMENT UPDATE------------    

                %Predict new measurement z (may differ from actual measurement) and get
                %jacobian H, and measurement residual model
                [z_new_hat, H_new] = EKF_3dQuad_funcs.meas_predict(x_new_hat, T_rc2rq);
            
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
                S_new_hat = H_new * P_new_hat * transpose(H_new) + W;

        
        %*************************************************
        %------------- STEP 3: STATE UPDATE -------------- 
                
                %Calculate Kalman gain
                K_new = P_new_hat * transpose(H_new)/(S_new_hat);
            
                %Update state est
                x_new = x_new_hat + (K_new * y_new);
            
                %Update state covariance
                I = eye(size(H_new,2), size(H_new,2)); %make identity matrix of appropriate size
                P_new = (I - K_new * H_new) * P_new_hat;

            end

            %Enforce quaternion constraints - closest quaternions
            if dot(x_k(4:7), x_new(4:7)) < 0
                x_new(4:7) = -x_new(4:7);
            end
            %normalise orientation quaternion
            x_new(4:7,1) =x_new(4:7,1)/norm(x_new(4:7,1));

        
        end
      
      %------------------------------------------------------------%
 
      function [x_new_hat, F_new_hat, L_new_hat] = dyn_update(g, x_k, u_new, t_delta, T_imu2rq, integ)
        %DYN_UPDATE Predict a priori state using dynamics and IMU
        %    Detailed explanation goes here
        %     
        %    Inputs:
        %        x_k - a posteriori state est (at k): x_k= [x; y; z; q_w; q_x; q_y; q_z; x_dot; y_dot; z_dot; ba_x; ba_y; ba_z; bg_x; bg_y; bg_z];
        %        u_new - IMU as pseudo control input vector at k+1: u_new = [u_gx; u_gy; u_gz; u_ax; u_ay; u_az]         
        %        t_delta - time since last EKF loop, in seconds
        %        T_imu2rq - homog transformation matrix from IMU to body frame (rq - "real quad")
        %        reset - flag to recalculate jacobians (1) or not (0)
        %        integ - integration type: 'rect' or 'trap'
        %    Outputs:
        %        x_next - a priori state est for k+1
        %        F_k - process model jacobian w.r.t. state evaluated at k
        %        L_k - process model jacobian w.r.t. process noise eval at k
        %        proTerm_k - state innovation term

        % *****  Initialisations  *****
            if size(x_k, 1)==10
                 w_k = zeros(6,1); %process noise assumed to be zero-mean gaussian
            else
                 w_k = zeros(12,1); %process noise assumed to be zero-mean gaussian
            end
            
            persistent u_k;
            if isempty(u_k)
                integ = 'rect'; %run rectangular integration on first run
            end

            %Extract variables to match symbolic toolbox output
            if integ == trap
                numerics = [x_k; u_new; u_k; w_k; t_delta; g];
            else
                numerics = [x_k; u_new; w_k; t_delta; g];
            end

        % ***** Evaluate model and jacobians*****


            %Run dynamics update
            x_new_hat = feval(strcat('ekf_processModel_', integ), numerics);
            F_new_hat = feval(strcat('ekf_F_', integ), numerics);
            L_new_hat = feval(strcat('ekf_L_', integ), numerics);

            % Check quaternion term
            %Enforce Smallest angle change
            if dot(x_k(4:7), x_new_hat(4:7)) < 0
                x_new_hat(4:7) = -x_new_hat(4:7);
            end
            %Normalise
            x_new_hat(4:7) = x_new_hat(4:7) / norm(x_new_hat(4:7)); 

            %save u_k for next run
            u_k = u_new;
            
        end
           
      %------------------------------------------------------------%
      
      function [z_new_hat, H_new_hat] = meas_predict(x_new_hat, T_rc2rq)
    
          z_new_hat = feval('ekf_measModel', x_new_hat); %get predicted measurement
          H_new_hat = feval('ekf_H', x_new_hat); %and covariance
                                     
        end

      %------------------------------------------------------------%
        
      function calcProcessModel(integ, g, T_imu2rq)
        %CALCPROCESSMODEL Generates process model and jacobian functions using symbolic toolbox

            %general veriables
            q_imu2rq = rotm2quat(T_imu2rq(1:3, 1:3))';
            t_imu2rq = T_imu2rq(1:3, 4);           
                    
            %Define symbolic variables

            %general
            dt = sym("dt");
          
            %old state
            p = sym("p", [3,1]);
            v = sym("v", [3,1]);
            q = sym("q", [4,1]);
            ba = sym("ba", [3,1]);
            bg = sym("bg", [3,1]);
            x = [p; q; v; ba; bg];

            %new "control input"
            u_g_new = sym("u_g_new", [3,1]);
            u_a_new = sym("u_a_new", [3,1]);
            %u_q = sym("u_q", [4,1]);
            %u = [u_g; u_a; u_q];
            u_new = [u_g_new; u_a_new];

            %process noise
            w_g = sym("w_g", [3,1]);
            w_a = sym("w_a", [3,1]);
            w_bg = sym("w_bg", [3,1]);
            w_ba = sym("w_ba", [3,1]);
            w = [w_g; w_a; w_ba; w_bg];

            %measurements (cam)
            p_c  = sym("p_c", [3,1]);
            theta_c = sym("theta_c", [3,1]);
            z = [p_c; theta_c];

            %prev "control input"
            u_g_old = sym("u_g_prev", [3,1]);
            u_a_old = sym("u_a_prev", [3,1]);
            u_old = [u_g_old; u_a_old];

            %compile symbols
            symbols_r = [x; u_new; w; dt];
            symbols_t = [x; u_new; u_old; w; dt];

            %rotations and conversions

            %universal
            R_imu2rq = [1 - 2*(q_imu2rq(3)^2 + q_imu2rq(4)^2), 2*(q_imu2rq(2)*q_imu2rq(3) - q_imu2rq(4)*q_imu2rq(1)), 2*(q_imu2rq(2)*q_imu2rq(4) + q_imu2rq(3)*q_imu2rq(1)); % Convert q to rotation matrix
                2*(q_imu2rq(2)*q_imu2rq(3) + q_imu2rq(4)*q_imu2rq(1)), 1 - 2*(q_imu2rq(2)^2 + q_imu2rq(4)^2), 2*(q_imu2rq(3)*q_imu2rq(4) - q_imu2rq(2)*q_imu2rq(1));
                2*(q_imu2rq(2)*q_imu2rq(4) - q_imu2rq(3)*q_imu2rq(1)), 2*(q_imu2rq(3)*q_imu2rq(4) + q_imu2rq(2)*q_imu2rq(1)), 1 - 2*(q_imu2rq(2)^2 + q_imu2rq(3)^2)];
            lmo_rq2rw = [q(1) -q(2) -q(3) -q(4) %turn q into left matrix operator for easier math
                        q(2) q(1) -q(4) q(3)
                        q(3) q(4) q(1) -q(2)
                        q(4) -q(3) q(2) q(1)];           
            R_rq2rw = [1 - 2*(q(3)^2 + q(4)^2), 2*(q(2)*q(3) - q(4)*q(1)), 2*(q(2)*q(4) + q(3)*q(1)); % Convert q to rotation matrix
                2*(q(2)*q(3) + q(4)*q(1)), 1 - 2*(q(2)^2 + q(4)^2), 2*(q(3)*q(4) - q(2)*q(1));
                2*(q(2)*q(4) - q(3)*q(1)), 2*(q(3)*q(4) + q(2)*q(1)), 1 - 2*(q(2)^2 + q(3)^2)];
            
            %***** RECTANGULAR *****
            if integ == 'rect'
                % define state changes - rectangular
                p_dot = v;
                w_Q = R_imu2rq * (u_g_new - bg + w_g); %get current angular accel in quad frame
                q_u = [0; w_Q]; %turn gyro reading into a quaternion - it is a rate! Don't normalise
                %q_dot = 0.5 * lmo_rq2rw * q_u;
                v_dot = ((R_rq2rw * R_imu2rq *  (u_a_new - ba + w_a)) + g);
                ba_dot = w_ba;
                bg_dot = w_bg;
                %or, incremental change
                dangle = norm(w_Q) * dt;  %incremental change for non linear quaternion update

                w_Q_hat = w_Q/norm(w_Q);
                dq = [cos(dangle/2); sin(dangle/2)*w_Q_hat];
                dq_rmo = [dq'; 
                          -dq(2) dq(1) -dq(4) dq(3);
                          -dq(3) dq(4) dq(1) -dq(2);
                          -dq(4) -dq(3) dq(2) dq(1)];
            
                % define process model  
                p_new = p + dt * p_dot;
                v_new = v + dt * v_dot;
                ba_new = ba + dt * ba_dot;
                bg_new = bg + dt * bg_dot;
                %q_new = q + dt*q_dot; % linearisation option (legacy)
                %q_new = q * dq_rmo;
                q_new = lmo_rq2rw * dq;

                symbols = symbols_r;
            

            %***** TRAPEZOIDAL *****
            elseif integ == 'trap'
                %update state in very specific order

                %first biases
                ba_dot = w_ba;
                bg_dot = w_bg;
                ba_new = ba + dt * ba_dot;
                bg_new = bg + dt * bg_dot;

                %quaternion update - use average angular rate, but don't try to average the current pose
                w_Q_av = R_imu2rq * ( 0.5*(u_g_old + u_g_new) - bg + w_g); %get average angular rate
                dangle = norm(w_Q_av) * dt; %get incremental angle change
                %update attitude quaternion
                w_Q_hat = w_Q_av/norm(w_Q_av);
                dq = [cos(dangle/2); sin(dangle/2)*w_Q_hat];
                dq_rmo = [dq'; 
                          -dq(2) dq(1) -dq(4) dq(3);
                          -dq(3) dq(4) dq(1) -dq(2);
                          -dq(4) -dq(3) dq(2) dq(1)];
                q_new = lmo_rq2rw * dq;

                %velocity update
                R_rq2rw_new = [1 - 2*(q_new(3)^2 + q_new(4)^2), 2*(q_new(2)*q_new(3) - q_new(4)*q_new(1)), 2*(q_new(2)*q_new(4) + q_new(3)*q_new(1)); % Convert q_new to rotation matrix
                            2*(q_new(2)*q_new(3) + q_new(4)*q_new(1)), 1 - 2*(q_new(2)^2 + q_new(4)^2), 2*(q_new(3)*q_new(4) - q_new(2)*q_new(1));
                            2*(q_new(2)*q_new(4) - q_new(3)*q_new(1)), 2*(q_new(3)*q_new(4) + q_new(2)*q_new(1)), 1 - 2*(q_new(2)^2 + q_new(3)^2)];
                a_old = ((R_rq2rw * R_imu2rq *  (u_a_old - ba + w_a)) + g);
                a_new = ((R_rq2rw_new * R_imu2rq *  (u_a_new - ba_new + w_a)) + g);
                a_av = 0.5 * (a_old + a_new);
                v_new = v + dt*a_av;

                %position
                v_av = 0.5 * (v_new + v);
                p_new = p + dt*v_av;

                symbols = symbols_t;
                
            end
            
            %***** define complete process model - 16 element version ****
            x_new_16el = [p_new; q_new; v_new; ba_new; bg_new]; 

            % define jacobians                      
            F_star_16el = jacobian(x_new_16el, x); 
            L_star_10el = jacobian(x_new_16el, w);

            % generate and save functions
            matlabFunction(F_star_16el, 'File', strcat('Sandbox/ekf_F_16el_', integ), 'Vars', {symbols});
            matlabFunction(L_star_10el, 'File', strcat('Sandbox/ekf_L_16el_', integ), 'Vars', {symbols});
            matlabFunction(x_new_16el, 'File', strcat('Sandbox/ekf_processModel_16el_', integ), 'Vars', {symbols});

            %***** define complete process model - 10 element version ****
            x_new_10el = [p_new; q_new; v_new]; 
            x_new_10el = subs(x_new_10el, [ba; bg; w_bg; w_ba], zeros(4*3, 1)); %simplify by evaluating with biases and associated noise = 0

            % define jacobians                      
            F_star_10el = jacobian(x_new_10el, x(1:10)); 
            L_star_10el = jacobian(x_new_10el, w(1:6));

            % generate and save functions
            matlabFunction(F_star_10el, 'File', strcat('Sandbox/ekf_10el_', integ), 'Vars', {symbols});
            matlabFunction(L_star_10el, 'File', strcat('Sandbox/ekf_L_10el_', integ), 'Vars', {symbols});
            matlabFunction(x_new_10el, 'File', strcat('Sandbox/ekf_processModel_10el_', integ), 'Vars', {symbols});

        end

      %------------------------------------------------------------%
      
      function calcMeasurementModel(T_rc2rq)
            %Function using symbolic toolbox to calculate the matrices involved in
            %the CAMERA measurement model.
           
            %Extract useful variables
            R_rc2rq = (T_rc2rq(1:3, 1:3));
            t_rc2rq = T_rc2rq(1:3, 4);
            q_rc2rq = rotm2quat(R_rc2rq)';
  
            %Define symbolic variables
            
            %state
            p = sym("p", [3,1]);
            v = sym("v", [3,1]);
            q = sym("q", [4,1]);
            ba = sym("ba", [3,1]);
            bg = sym("bg", [3,1]);
            x = [p; q; v; ba; bg];

            %compile used symbols
            symbols = x; %state only
           
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
            measurementModel = [p_z; q_z]; %predicted measurement vector (symbolic)
            H_star = jacobian(measurementModel, x); %jacobian of measurement wrt state

            % generate and save functions
            h = matlabFunction(measurementModel, 'File', strcat('Sandbox/ekf_measModel'), 'Vars', {symbols});
            H_star_func = matlabFunction(H_star, 'File', strcat('Sandbox/ekf_H'), 'Vars', {symbols});

        end

      %------------------------------------------------------------%

        function [ekfResult, P_0, Q, W, t_delta] = initEKF(startTime, x_0)

            %Define P_k - State covariance
            P_0 = diag([0.001, 0.001, 0.001, ...
            0.001, 0.001, 0.001, 0.001, ...
            0.001, 0.001, 0.001, ...
            0.2, 0.2, 0.2, ...
            0.01, 0.01, 0.01]); %Initial, 16 el

            %process noise covariance (noise space)            
            %t_delta_dur = ((u_timeHist(1,2) - u_timeHist(1,1)));
            %t_delta = milliseconds(t_delta_dur) * 0.001;
            t_delta = 0.0623;  %average data rate
            w_g = [(1.037e-03)^2, 0.001^2, 0.00129^2]' / t_delta; %from Allan variance
            w_g = [0.0029^2, 0.005^2, 0.0037^2]';
            w_a = [(5.809e-03)^2, 0.008^2, 0.011^2]' / t_delta; %from Allan variance
            w_a = [0.0128^2, 0.0164^2, 0.0191^2 ]';
            w_ba = [(1.081e-05)^2, (0.000001)^2, (6.55e-06)^2]' /t_delta; %from Allan variance
            w_bg = [(1e-07)^2, (1e-07)^2, (1e-07)^2]' /t_delta; %from Allan variance
            Q = diag([w_g; w_a; w_ba; w_bg]);

            Q = diag([0.01, 0.01, 0.01, 0.2, 0.2, 0.2, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001]); %16 el - [w_g, w_acc, w_ba, w_ba]
        
            %and measurement covariance
            W =diag([0.1506^2, 0.1506^2, 0.1506^2, 0.01, 0.007897, 0.007897, 0.007897]);  

            
            %initialise EKF output history (do it after first state to get sizes right)
            ekfResult = struct();
            ekfResult.time = createArray(1, 0, 'datetime');
            ekfResult.time = datetime(ekfResult.time, 'Format', 'yyyyMMdd_HHmmss_SSS');
            ekfResult.stateEst = createArray(size(x_0, 1), 0, 'double');
            ekfResult.P = createArray(size(x_0, 1), size(x_0, 1), 0, 'double');
            ekfResult.processTerm = createArray(size(x_0, 1), 0, 'double');
            ekfResult.measPred = createArray(7, 0, 'double');
            ekfResult.statePred = createArray(size(ekfResult.stateEst), 'double');
            ekfResult.z = createArray(7,0, 'double');
            ekfResult.input = createArray(6, 0, 'double');
            ekfResult.error = createArray(2, 0, 'double'); %position, angle
            ekfResult.Rt = zeros(3, 4, 0); %for plotting
            ekfResult.zError = createArray(2, 0, 'double'); %position, angle
            ekfResult.zHist = createArray(8, 0, 'double'); %include elapsed time in first position
            ekfResult.zInHist = createArray(8, 0, 'double'); %include elapsed time in first position
            ekfResult.elapsedTime = createArray(1,0, 'double');
            ekfResult.timeSinceLastCorrection = createArray(1,0, 'double');

            %Set first values
            ekfResult.stateEst(:,1) = x_0;
            ekfResult.time(:,1) = datetime(startTime, 'Format', 'yyyyMMdd_HHmmss_SSS');
            ekfResult.elapsedTime(1,1) = 0;
            ekfResult.P(:,:,1) = P_0;
            ekfResult.processTerm(:,1) = zeros(size(x_0));
            ekfResult.measPred(:,1) = zeros(7,1);
            ekfResult.statePred(:,1) = zeros(size(x_0));
            ekfResult.z(:,1) = NaN(7,1);
            ekfResult.input(:,1) = zeros(6,1);
            ekfResult.timeSinceLastCorrection(1,1) = 0;


            
        end
    end
    
end