classdef EKF_3dQuad_funcs
    methods (Static)

        %------------------------------------------------------------%
        function [x_new, P_new, processTerm_k, x_new_hat, z_new_hat] = EKF_loop(x_k, P_k, u_k, Q, z_k, W, t_delta, Rt_imu2rq, Rt_rc2rq)
        %Extended Kalman filter for a 3D quad. Must run iteratively
        %for each time step.
        
        % USAGE: 
        
        %
        % INPUTS:
        %     z_k - newly taken measurement (z_k)
        %     x_k - state est from previous time step (x_k)
        %     P_k - state covariance from previous time step
        %     u_k - control input from previous time step (u_k)
        %     Q - process noise covariance in the noise space
        %     W - measurement noise covariance in the state space
        %     ***convert to noise space version? 
        
        %
        % OUTPUTS:
        %     x_new - state est from previous time step (x_k-1)
        %     P_new - state covariance from previous time step
        
        
        %*************************************************
        %----------- STEP 0: INITIALISATIONS -------------
        
        % %Biases
         b_a = [0;0;0];
         b_g = [0;0;0];
        % 
        % %Process noise covariance (in the noise space)
        % sigma_state = [sigma_x, sigma_z, sigma_phi, sigma_xdot, sigma_zdot];
        % Q = diag(sigma_state);
        % 
        % % %Input noise Jacobian (noise influence matrix)
        % % L = L_update();
        % 
        % %Measurement noise covariance
        % sigma_meas = [sigma_xc, sigma_zc, sigma_thetac];
        % Z = diag(sigma_meas);
        
        
        
        
        %*************************************************
        %----------- STEP 1: DYNAMICS UPDATE -------------
        
            %Predict new state (a priori) and get prev jacobian 
            [x_new_hat, F_k, L_k, processTerm_k] = EKF_3dQuad_funcs.dyn_update(x_k, u_k, t_delta, Rt_imu2rq);
            
            %Predict new state covariance (in state space)
            P_new_hat = F_k * P_k * transpose(F_k) + L_k * Q * transpose(L_k);
        


        %*************************************************
        %ONLY RUN CORRECTION IF A NEW MEASUREMENT HAS BEEN DETECTED

            if isnan(z_k)
                x_new = x_new_hat;
                P_new = P_new_hat;
                z_new_hat = zeros(7,1);
            else
        %---------- STEP 2: MEASUREMENT UPDATE------------    

                %Predict new measurement z (may differ from actual measurement) and get
                %jacobian H
                [z_new_hat, H_new] = EKF_3dQuad_funcs.meas_predict(x_new_hat, Rt_rc2rq);
            
                %Calculate measurement residual y
                y_new = z_k - z_new_hat;
            
                %Compute predicted measurement covariance S
                S_new_hat = H_new * P_new_hat * transpose(H_new) + W;

        
        %*************************************************
        %------------- STEP 3: STATE UPDATE -------------- 
                
                %Calculate Kalman gain
                K_new = P_new_hat * transpose(H_new)/(S_new_hat);
            
                %Update state est
                x_new = x_new_hat + (K_new * y_new);
            
                %Update state covariance
                I =  eye(size(H_new,2), size(H_new,2)); %make identity matrix of appropriate size
                P_new = (I - K_new * H_new) * P_new_hat;
            end

            %normalise orientation quaternion
            q_new = x_new(4:7,1);
            q_new_unit = q_new/norm(q_new);
            x_new(4:7,1)=q_new_unit;
        
        end

   %------------------------------------------------------------%

   function [x_next_hat, F_k, L_k, proTerm_k] = dyn_update(x_k, u_k, t_delta, Rt_imu2rq)
            %F_DYN_2DQUAD_NL Summary of this function goes here
            %   Detailed explanation goes here
            
            % USAGE: 
            
            %
            % INPUTS:
            %     x_k -   state vector for time k: 
            %               x_k= [x; y; z; q_w; q_x; q_y; q_z; x_dot; y_dot; z_dot]
            %     u_k - control input vector for current time step k: 
            %               u_k = [x, ] 
            
            %
            % OUTPUTS:
            %     x_next - state est for next time step, based on dynamics
            %     x_cov_k - state covariance for current time step

            %% Initialisations
            %g= [0; -9.81];
            %FAKE IMU DOES NOT READ g:
            g = [0; 0; 0];


            %% CALC MODEL EVERY TIME
            % Calculate model
            [processDE, F_star, L_star, R_star, symbols] = EKF_3dQuad_funcs.calcProcessModel(Rt_imu2rq);

            % Extract useful variables
            q_k = x_k(4:7, 1);
                        

            %Extract variables to match symbolic toolbox output
            numerics = [x_k; u_k];

            % Predict next state

            % next state
            processDE_num = (double(subs(processDE, symbols, numerics)));
            proTerm_k =  processDE_num;
            
            x_next_hat = x_k + t_delta * processDE_num;

            % process covariance
            F_k = double(subs(F_star, symbols, numerics));

            %And L, the input noise covariance: will be used to transform
            %covariance in the noise space into the state space. Also
            %called the "noise influence matrix"
            L_k = double(subs(L_star, symbols, numerics));

            R_k = double(subs(R_star, symbols, numerics));

        end
           
  %------------------------------------------------------------%
        function [z_new_hat, H_new] = meas_predict(x_new_hat, Rt_rc2rq)
        
            %Get measurement model and associated covariance (symbolic)
            [measModel, H_star, symbols] = EKF_3dQuad_funcs.calcMeasurementModel(Rt_rc2rq);

            %set up values to substitute
            numerics = x_new_hat;

            %Get predicted measurement
            z_new_hat = double(subs(measModel, symbols, numerics));
            
            %Get predicted measurement covariance
            H_new = double(subs(H_star, symbols, numerics));

        
        end

  %------------------------------------------------------------%
        
        function [processDE, F_star, L_star, R_rw2rq, symbols] = calcProcessModel(Rt_imu2rq)
        %Function using symbolic toolbox to calculate the matrices involved in
        %the quad process model.
        
            g = [0; 0; -9.81];  %world frame
            R_imu2rq =Rt_imu2rq(1:3, 1:3);
            t_imu2rq =Rt_imu2rq(1:3, 4);
            q_imu2rq = (rotm2quat(R_imu2rq));
            q_imu2rq = transpose(q_imu2rq);
                    
            %Define symbolic variables

            %state
            p = sym("p", [3,1]);
            v = sym("v", [3,1]);
            q = sym("q", [4,1]);
            x = [p; q; v];

            %"control input"
            u_g = sym("u_g", [3,1]);
            u_a = sym("u_a", [3,1]);
            u = [u_g; u_a];

            %measurements (cam)
            p_c  = sym("p_c", [3,1]);
            theta_c = sym("theta_c", [3,1]);
            z = [p_c; theta_c];

            %quaternion right matrix operator
             q_u = [0; u_g]; %turn gyro reading into a quaternion - it is a rate! Don't normalise
             rmo_q_u = [q_u(1) -q_u(2) -q_u(3) -q_u(4);
                        q_u(2) q_u(1) q_u(4) -q_u(3);
                        q_u(3) -q_u(4) q_u(1) q_u(2);
                        q_u(4) q_u(3) -q_u(2) q_u(1)];

            %will also need left matrix operator of q
            lmo_q = [q(1) -q(2) -q(3) -q(4)
                    q(2) q(1) -q(4) q(3)
                    q(3) q(4) q(1) -q(2)
                    q(4) -q(3) q(2) q(1)];

             


            %rotation matrix for orientation quaternion
            % R = [1 - 2*q(3)^2 - 2*q(4)^2, 2*q(2)*q(3) - 2*q(1)*q(4), 2*q(2)*q(4) + 2*q(1)*q(3);
            %      2*q(2)*q(3) + 2*q(1)*q(4), 1 - 2*q(2)^2 - 2*q(4)^2, 2*q(3)*q(4) - 2*q(1)*q(2);
            %      2*q(2)*q(4) - 2*q(1)*q(3), 2*q(3)*q(4) + 2*q(1)*q(2), 1 - 2*q(2)^2 - 2*q(3)^2];
            
            %R = [2*(q(1)^2 + q(2)^2)-1, 2*q(2)*q(3) - 2*q(1)*q(4), 2*(q(2)*q(4) + q(1)*q(3));
            %    2*(q(2)*q(3) + q(1)*q(4)), 2*(q(1)^2 + q(3)^2)-1, 2*(q(3)*q(4)-q(1)*q(2));
            %    2*(q(2)*q(4)-q(1)*q(3)), 2*(q(3)*q(4) + q(1)*q(2)), 2*(q(1)^2 + q(4)^2)-1];

            R_rw2rq = [1 - 2*(q(3)^2 + q(4)^2), 2*(q(2)*q(3) - q(4)*q(1)), 2*(q(2)*q(4) + q(3)*q(1));
                2*(q(2)*q(3) + q(4)*q(1)), 1 - 2*(q(2)^2 + q(4)^2), 2*(q(3)*q(4) - q(2)*q(1));
                2*(q(2)*q(4) - q(3)*q(1)), 2*(q(3)*q(4) + q(2)*q(1)), 1 - 2*(q(2)^2 + q(3)^2)];
            

            %Manual quaternion rotation
            q_u_rq = [q_imu2rq(1)*q_u(1) - q_imu2rq(2)*q_u(2) - q_imu2rq(3)*q_u(3) - q_imu2rq(4)*(q_u(4));
                       q_imu2rq(1)*q_u(2) + q_u(1)*q_imu2rq(2) + q_imu2rq(3)*q_u(4) - q_u(3)*q_imu2rq(4);
                       q_imu2rq(1)*q_u(3) + q_u(1)*q_imu2rq(3) + q_u(2)*q_imu2rq(4) - q_imu2rq(2)*q_u(4);
                       q_imu2rq(1)*q_u(4) + q_u(1)*q_imu2rq(4) + q_imu2rq(2)*q_u(3) - q_u(2)*q_imu2rq(3)];

            q_u_rw = [q(1)*q_u_rq(1) - q(2)*q_u_rq(2) - q(3)*q_u_rq(3) - q(4)*(q_u_rq(4));
                       q(1)*q_u_rq(2) + q_u_rq(1)*q(2) + q(3)*q_u_rq(4) - q_u_rq(3)*q(4);
                       q(1)*q_u_rq(3) + q_u_rq(1)*q(3) + q_u_rq(2)*q(4) - q(2)*q_u_rq(4);
                       q(1)*q_u_rq(4) + q_u_rq(1)*q(4) + q(2)*q_u_rq(3) - q_u_rq(2)*q(3)];


            %If IMU is not at drone centre, the rotational component of the
            %acceleration must be removed
            %assuming omega_dot is negligible
            omega_dot = [0 ; 0; 0];
            a_rot = cross(omega_dot, t_imu2rq) + cross(u_g, cross(u_g, t_imu2rq));

            % processDE = [v;
            %             0.5 * Omega * q;
            %             R * u_a + g];
            omega_rq = 0.5 * q_u_rw;
            processDE = [v;
                         0.5* lmo_q * (rmo_q_u * q_imu2rq);
                         (transpose(transpose(R_rw2rq) *R_imu2rq) * u_a) + g];

            F_star = jacobian(processDE, x); 

            L_star = jacobian(processDE, u);

            symbols = [x; u];

        end

  %------------------------------------------------------------%
        function [measurementModel, H_star, symbols] = calcMeasurementModel(Rt_rc2rq)
            %Function using symbolic toolbox to calculate the matrices involved in
            %the CAMERA measurement model.
           
            %Extract useful variables
            R_rc2rq = (Rt_rc2rq(1:3, 1:3));
            t_rc2rq = Rt_rc2rq(1:3, 4);
            q_rc2rq = rotm2quat(R_rc2rq);

            %Define symbolic variables
            
            %state
            p = sym("p", [3,1]);
            v = sym("v", [3,1]);
            q = sym("q", [4,1]);
            x = [p; q; v];

            %Extract rotation matrix
            R_rq2rw = [1 - 2*(q(3)^2 + q(4)^2), 2*(q(2)*q(3) - q(4)*q(1)), 2*(q(2)*q(4) + q(3)*q(1));
                2*(q(2)*q(3) + q(4)*q(1)), 1 - 2*(q(2)^2 + q(4)^2), 2*(q(3)*q(4) - q(2)*q(1));
                2*(q(2)*q(4) - q(3)*q(1)), 2*(q(3)*q(4) + q(2)*q(1)), 1 - 2*(q(2)^2 + q(3)^2)];
                      

            %Manual quaternion rotation: q*q
            q_rc2rw = [q(1)*q_rc2rq(1) - q(2)*q_rc2rq(2) - q(3)*q_rc2rq(3) - q(4)*(q_rc2rq(4));
                       q(1)*q_rc2rq(2) + q_rc2rq(1)*q(2) + q(3)*q_rc2rq(4) - q_rc2rq(3)*q(4);
                       q(1)*q_rc2rq(3) + q_rc2rq(1)*q(3) + q_rc2rq(2)*q(4) - q(2)*q_rc2rq(4);
                       q(1)*q_rc2rq(4) + q_rc2rq(1)*q(4) + q(2)*q_rc2rq(3) - q_rc2rq(2)*q(3)];

            %measurements (cam)
            p_z = R_rq2rw * t_rc2rq + p;
            q_z = q_rc2rw;
            
        
            %try with quaternions
            measurementModel = [p_z; q_z];

            H_star = jacobian(measurementModel, x);

            symbols = x;

        end

    end
    
end