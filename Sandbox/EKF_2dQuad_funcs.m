classdef EKF_2dQuad_funcs
    methods (Static)

        %------------------------------------------------------------%
        function [x_new, P_new] = EKF_loop(x_k,P_k, u_k, Q, z_new, W, t_delta)
        %KALMAN_QUAD2D extended Kalman filter for a 2D quad. Must run iteratively
        %for each time step.
        
        % USAGE: 
        
        %
        % INPUTS:
        %     z_new - newly taken measurement (z_new)
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
         b_a = [0;0];
         b_g = 0;
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
            [x_new_hat, F_k, R_k, G_k, L_k] = EKF_2dQuad_funcs.dyn_update(x_k, u_k, b_g, b_a, t_delta);
            
            %Predict new state covariance (in state space)
            P_new_hat = F_k * P_k * transpose(F_k) + L_k * Q * transpose(L_k);
        


        %*************************************************
        %ONLY RUN CORRECTION IF A NEW MEASUREMENT HAS BEEN DETECTED

            if isnan(z_new)
                x_new = x_new_hat;
                P_new = P_new_hat;
            else
        %---------- STEP 2: MEASUREMENT UPDATE------------    

                %Predict new measurement z (may differ from actual measurement) and get
                %jacobian H
                [z_new_hat, H_new] = EKF_2dQuad_funcs.meas_predict(x_new_hat);
            
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
                I =  eye(size(H_new,2), size(H_new,2)); %make identity matrix of appropriate size
                P_new = (I - K_new * H_new) * P_new_hat;
            end
        
        end

   %------------------------------------------------------------%

        function [x_next, F_k, R_k, G_k, L_k] = dyn_update(x_k, u_k, b_g, b_a, t_delta)
            %F_DYN_2DQUAD_NL Summary of this function goes here
            %   Detailed explanation goes here
            
            % USAGE: 
            
            %
            % INPUTS:
            %     x_k -   state vector for time k: 
            %               x_k= [x; z; phi; x_dot; z_dot; q_dot]
            %     u_k - control input vector for current time step k: 
            %               u_k = [x, ] 
            
            %
            % OUTPUTS:
            %     x_next - state est for next time step, based on dynamics
            %     x_cov_k - state covariance for current time step

            %g= [0; -9.81];
            %FAKE IMU DOES NOT READ g:
            g = [0; 0];

            % Extract useful values
            phi_k = x_k(3);
            phi_ddot_m_k = u_k(1);
            x_ddot_m_k = u_k(2);
            z_ddot_m_k = u_k(3);

            %Rotation matrix from IMU frame to world frame
            % R_k = [cos(phi_k) -sin(phi_k);
            %        sin(phi_k) cos(phi_k)];
            % G_k = [1];
            % G_k_inv = inv(G_k);
            %IMU IN WORLD FRAME:
            R_k = [1 0; 0 1];
            G_k = [1];
            G_k_inv = [1];

            
            A = [0 0 0 1 0;
                 0 0 0 0 1;
                 0 0 0 0 0;
                 0 0 0 0 0;
                 0 0 0 0 0];
            B = [0 0 0;
                 0 0 0;
                 G_k_inv 0 0;
                 zeros(2,1), R_k];
            C = [zeros(2,3);
                -G_k_inv 0 0;
                 zeros(2,1), -R_k];
            D = [0; 0; -G_k_inv*b_g; (-R_k*b_a)+g];
            

            % Estimate next state
            x_next = x_k + t_delta*(A*x_k + B*u_k + D);

            %while we're here, calculate prev covariance
            % F_k = [0 0 0 1 0;
            %        0 0 0 0 1;
            %        0 0 (sin(phi_k)/cos(phi_k))*(phi_ddot_m_k-b_g) 0 0;
            %        0 0 (-sin(phi_k)*(x_ddot_m_k-b_a(1)) - cos(phi_k)*(z_ddot_m_k-b_a(2))) 0 0;
            %        0 0 (cos(phi_k)*(x_ddot_m_k-b_a(1)) - sin(phi_k)*(z_ddot_m_k-b_a(2))) 0 0 ];
            %FOR THEORETICAL IMU MAGICALLY READING IN WORLD FRAME
            F_k = [0 0 0 1 0;
                   0 0 0 0 1;
                   zeros(3,5)];

            %And L, the input noise covariance: will be used to transform
            %covariance in the noise space into the state space. Also
            %called the "noise influence matrix"
            L_k = C;
            

        end

  %------------------------------------------------------------%
        function [z_new_hat, H_new] = meas_predict(x_new_hat)
        
            z_new_hat = x_new_hat(1:3, 1);
            
            H_new = [1 0 0 0 0;
                      0 1 0 0 0;
                      0 0 1 0 0];
        
        end

  %------------------------------------------------------------%
    end
    
end