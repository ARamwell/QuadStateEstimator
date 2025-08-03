classdef EKF_3dQuad_10el_funcs
    methods (Static)

        %------------------------------------------------------------%
        function [x_new, P_new, processTerm_k, x_new_hat, z_new_hat, z_k] = EKF_loop(g, x_k, P_k, u_k, Q, z_k, W, t_delta, Rt_imu2rq, Rt_rc2rq, reset)
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
            [x_new_hat, F_k, L_k, processTerm_k] = EKF_3dQuad_10el_funcs.dyn_update_trap(g, x_k, u_k, t_delta, Rt_imu2rq, reset);

            %enforce quaternion continuity
            if dot(x_new_hat(4:7), x_k(4:7)) < 0
                x_new_hat(4:7) = -x_new_hat(4:7);
            end

                       
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
                [z_new_hat, H_new] = EKF_3dQuad_10el_funcs.meas_predict(x_new_hat, Rt_rc2rq, z_k);
            
                %Calculate measurement residual y
                %Enforce quaternion constraints - closest quaternions
                if dot(x_k(4:7), z_k(4:7)) < 0
                    z_k(4:7) = -z_k(4:7);
                end
                if dot(z_k(4:7), z_new_hat(4:7)) < 0
                    z_new_hat(4:7) = -z_new_hat(4:7);
                end

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

   function [x_next_hat, F_k, L_k, proTerm_k] = dyn_update_old(g, x_k, u_k, t_delta, Rt_imu2rq, reset)
            %F_DYN_2DQUAD_NL Summary of this function goes here
            %   Detailed explanation goes here
            
            % USAGE: 
            
            %
            % INPUTS:
            %     x_k -   state vector for time k: 
            %               x_k= [x; y; z; q_w; q_x; q_y; q_z; x_dot; y_dot; z_dot]
            %     u_k - control input vector for current time step k: 
            %               u_k = [w_x; w_y; w_z; a_x; a_y; a_z] 
            
            %
            % OUTPUTS:
            %     x_next - state est for next time step, based on dynamics
            %     x_cov_k - state covariance for current time step

            %% Initialisations
            if reset == 1
                clear persistent;
            end

            %% Persistent variables
            persistent F_star;
            persistent L_star;
            persistent processDE;
            persistent xdot_star;
            persistent xdot_num;
            persistent symbols;

            %Extract variables to match symbolic toolbox output
            numerics = [x_k; u_k; t_delta];


            %% CALC MODEL EVERY TIME
            %[processDE, F_star, L_star, R_star, xdot_star, symbols] = EKF_3dQuad_10el_funcs.calcProcessModel(g, Rt_imu2rq);
            
            if isempty(processDE) %if first iteration
                % Calculate model
                [processDE, F_star, L_star, xdot_star, R_star, symbols] = EKF_3dQuad_10el_funcs.calcProcessModel(g, Rt_imu2rq);
                %Use rectangular integration
                xdot_num = (double(subs(xdot_star, symbols, numerics)));
                x_next_hat = x_k + t_delta * xdot_num;
            else 
                % xdot_num = (double(subs(xdot_star, symbols, numerics)));
                % x_next_hat = x_k + t_delta * xdot_num;
                
                % Use precalculated model
                % AND Apply trapezoidal integration for a, v and omega

                % Trapezoidal integration requires cascaded approach

                % update orientation first - a bit geometrically
                % meaningless
                qdot_new =(double(subs(xdot_star(4:7), symbols, numerics)));
                qdot_old = xdot_num(4:7);
                qdot_trap = (qdot_old + qdot_new)/2;
                q_new = x_k(4:7) + t_delta * qdot_trap;
                numerics(4:7) = q_new; %update velocity in state

                % then velocity
                a_old = xdot_num(8:10);
                a_new = double(subs(xdot_star(8:10), symbols, numerics));
                a_trap = (a_new+a_old)/2;
                v_new = x_k(8:10) + t_delta * a_trap;
                numerics(8:10) = v_new; %update velocity in state

                % then position
                v_new = double(subs(xdot_star(1:3), symbols, numerics));
                v_old = xdot_num(1:3);
                v_trap = (v_new+v_old)/2;
                p_new = x_k(1:3) + t_delta * v_trap;
                numerics(1:3) = p_new; %update position


                %Add it all together
                xdot_num = [v_new; qdot_new; a_new];
                x_next_hat = x_k + t_delta * xdot_num;
                x_next_hat = [p_new; q_new; v_new];
            end
 
            % process covariance
            F_k = double(subs(F_star, symbols, numerics));

            %And L, the input noise covariance: will be used to transform
            %covariance in the noise space into the state space. Also
            %called the "noise influence matrix"
            L_k = double(subs(L_star, symbols, numerics));

            % %% Use precalculated model
            % p1 = x_k(1);
            % p2 = x_k(2);
            % p3 = x_k(3);
            % q1 = x_k(4);
            % q2 = x_k(5);
            % q3 = x_k(6);
            % q4 = x_k(7);
            % v1 = x_k(8);
            % v2 = x_k(9);
            % v3 = x_k(10);
            % u_g1 = u_k(1);
            % u_g2 = u_k(2);
            % u_g3 = u_k(3);
            % u_a1 = u_k(4);
            % u_a2 = u_k(5);
            % u_a3 = u_k(6);
            % 
            % processDE_num = [                                                                                       v1;
            %                                                                                                         v2;
            %                                                                                                         v3;
            %                                                                  - (q2*u_g1)/2 - (q3*u_g2)/2 - (q4*u_g3)/2;
            %                                                                    (q1*u_g1)/2 + (q3*u_g3)/2 - (q4*u_g2)/2;
            %                                                                    (q1*u_g2)/2 - (q2*u_g3)/2 + (q4*u_g1)/2;
            %                                                                    (q1*u_g3)/2 + (q2*u_g2)/2 - (q3*u_g1)/2;
            %                           u_a3*(2*q1*q3 + 2*q2*q4) - u_a2*(2*q1*q4 - 2*q2*q3) - u_a1*(2*q3^2 + 2*q4^2 - 1);
            %                           u_a1*(2*q1*q4 + 2*q2*q3) - u_a2*(2*q2^2 + 2*q4^2 - 1) - u_a3*(2*q1*q2 - 2*q3*q4);
            %                 u_a2*(2*q1*q2 + 2*q3*q4) - u_a1*(2*q1*q3 - 2*q2*q4) - u_a3*(2*q2^2 + 2*q3^2 - 1) - 981/100];
            % 
            %  F_k = [0, 0, 0,                     0,                                 0,                                 0,                                 0, 1, 0, 0;
            %         0, 0, 0,                     0,                                 0,                                 0,                                 0, 0, 1, 0;
            %         0, 0, 0,                     0,                                 0,                                 0,                                 0, 0, 0, 1;
            %         0, 0, 0,                     0,                           -u_g1/2,                           -u_g2/2,                           -u_g3/2, 0, 0, 0;
            %         0, 0, 0,                u_g1/2,                                 0,                            u_g3/2,                           -u_g2/2, 0, 0, 0;
            %         0, 0, 0,                u_g2/2,                           -u_g3/2,                                 0,                            u_g1/2, 0, 0, 0;
            %         0, 0, 0,                u_g3/2,                            u_g2/2,                           -u_g1/2,                                 0, 0, 0, 0;
            %         0, 0, 0, 2*q3*u_a3 - 2*q4*u_a2,             2*q3*u_a2 + 2*q4*u_a3, 2*q1*u_a3 + 2*q2*u_a2 - 4*q3*u_a1, 2*q2*u_a3 - 2*q1*u_a2 - 4*q4*u_a1, 0, 0, 0;
            %         0, 0, 0, 2*q4*u_a1 - 2*q2*u_a3, 2*q3*u_a1 - 4*q2*u_a2 - 2*q1*u_a3,             2*q2*u_a1 + 2*q4*u_a3, 2*q1*u_a1 + 2*q3*u_a3 - 4*q4*u_a2, 0, 0, 0;
            %         0, 0, 0, 2*q2*u_a2 - 2*q3*u_a1, 2*q1*u_a2 - 4*q2*u_a3 + 2*q4*u_a1, 2*q4*u_a2 - 4*q3*u_a3 - 2*q1*u_a1,             2*q2*u_a1 + 2*q3*u_a2, 0, 0, 0];
            % 
            %  L_k = [    0,     0,     0,                     0,                     0,                     0;
            %             0,     0,     0,                     0,                     0,                     0;
            %             0,     0,     0,                     0,                     0,                     0;
            %         -q2/2, -q3/2, -q4/2,                     0,                     0,                     0;
            %          q1/2, -q4/2,  q3/2,                     0,                     0,                     0;
            %          q4/2,  q1/2, -q2/2,                     0,                     0,                     0;
            %         -q3/2,  q2/2,  q1/2,                     0,                     0,                     0;
            %             0,     0,     0, - 2*q3^2 - 2*q4^2 + 1,     2*q2*q3 - 2*q1*q4,     2*q1*q3 + 2*q2*q4;
            %             0,     0,     0,     2*q1*q4 + 2*q2*q3, - 2*q2^2 - 2*q4^2 + 1,     2*q3*q4 - 2*q1*q2;
            %             0,     0,     0,     2*q2*q4 - 2*q1*q3,     2*q1*q2 + 2*q3*q4, - 2*q2^2 - 2*q3^2 + 1];

            %% Predict
            %x_next_hat = processDE_num;
            %x_next_hat = x_k + t_delta * xdot_num;

            %% Check quaternion term
            %Enforce Smallest angle change
            if dot(x_k(4:7), x_next_hat(4:7)) < 0
                x_next_hat(4:7) = -x_next_hat(4:7);
            end
                       
            x_next_hat(4:7) = x_next_hat(4:7) / norm(x_next_hat(4:7)); %Normalise
            
            %% Log
            proTerm_k =  xdot_num;

   end
    %------------------------------------------------------------%

   function [x_next_hat, F_k, L_k, proTerm_k] = dyn_update_trap(g, x_k, u_k, t_delta, Rt_imu2rq, reset)
            %F_DYN_2DQUAD_NL Summary of this function goes here
            %   Detailed explanation goes here

            % USAGE: 

            %
            % INPUTS:
            %     x_k -   state vector for time k: 
            %               x_k= [x; y; z; q_w; q_x; q_y; q_z; x_dot; y_dot; z_dot]
            %     u_k - control input vector for current time step k: 
            %               u_k = [w_x; w_y; w_z; a_x; a_y; a_z] 

            %
            % OUTPUTS:
            %     x_next - state est for next time step, based on dynamics
            %     x_cov_k - state covariance for current time step

            %% Initialisations
            % INITIALISATIONS
                    w_k = zeros(6,1); %process noise assumed to be zero-mean gaussian    
            % 
            %% USE PRECALCULATED MATRICES
                    [x_next_hat, F_k, L_k, proTerm_k] = EKF_3dQuad_10el_funcs.initProcessModelMatrices_trap(x_k, u_k, w_k, t_delta, g, reset);

            %% DERIVE PROCESS MODEL AT START

                % if reset == 1
                %     clear persistent;
                % end
                % 
                % %Extract variables to match symbolic toolbox output
                % numerics = [x_k; u_k; w_k; t_delta; g];
                % 
                % % Persistent variables
                % persistent F_star;
                % persistent L_star;
                % persistent processDE;
                % persistent xdot_star;
                % persistent xdot_num;
                % persistent x_prev;
                % persistent u_prev;
                % persistent symbols;
                % 
                % if isempty(u_prev) %if first iteration
                %     % Calculate model
                %     [processDE, F_star, L_star, xdot_star, R_star, symbols] = EKF_3dQuad_10el_funcs.calcProcessModel(g, Rt_imu2rq);
                % 
                %     %Use rectangular integration (no previous measurements
                %     %to average)
                %     xdot_num = (double(subs(xdot_star, symbols, numerics)));
                %     x_next_hat = x_k + t_delta * xdot_num;
                %     u_prev = u_k;
                %     x_prev = x_k;
                % else %do trap integration
                % 
                %     %angular velocity
                %     i_ug = size(x_k) + 1; %find index of gyro measurement
                %     ug_trap = (u_prev(1:3) + u_k(1:3))/2; %get average angular rate
                %     numerics(i_ug:(i_ug+2)) = ug_trap; %update angular rate in substitution matrix
                %     qdot_trap =(double(subs(xdot_star(4:7), symbols, numerics)));
                % 
                %     %v first
                %     v_new = double(subs(xdot_star(1:3), symbols, numerics));
                %     v_old = xdot_num(1:3);
                %     v_trap = (v_new+v_old)/2;
                %     %numerics(8:10) = pdot_trap; %update velocity in subsitution matrix
                %     %p_new = x_k(1:3) + t_delta * v_trap;
                %     %numerics(1:3) = p_new; %update position
                % 
                %     %acceleration
                %     a_old = xdot_num(8:10);
                %     a_new = double(subs(xdot_star(8:10), symbols, numerics));
                %     a_trap = (a_new+a_old)/2;
                % 
                %     %apply trapezoidal integration
                %     p_new = x_k(1:3) + t_delta * v_trap;
                %     %p_test = double(subs(processDE(1:3), symbols, numerics));
                % 
                %     q_new = x_k(4:7) + t_delta * qdot_trap;
                %     %q_test = double(subs(processDE(4:7), symbols, numerics));
                % 
                %     v_new = x_k(8:10) + t_delta * a_trap;
                % 
                %     %Add it all together
                %     xdot_num = [v_trap; qdot_trap; a_trap];
                %     x_next_hat = x_k + t_delta * xdot_num;
                % 
                % end
                % 
                % %IGNORE TRAPEZOIDAL RELATIONSHIPS IN THE JACOBIANS (MOSTLY)
                % % process covariance
                % F_k = double(subs(F_star, symbols, numerics));
                % 
                % %And L, the input noise covariance: will be used to transform
                % %covariance in the noise space into the state space. Also
                % %called the "noise influence matrix"
                % L_k = double(subs(L_star, symbols, numerics));
                % 
                % % Log
                % proTerm_k =  xdot_num;
                % u_prev = u_k; %prepare for trapezoidal integration of angular velocity
                % x_prev = x_k;


            %% Check quaternion term
            %Enforce Smallest angle change
            if dot(x_k(4:7), x_next_hat(4:7)) < 0
                x_next_hat(4:7) = -x_next_hat(4:7);
            end

            x_next_hat(4:7) = x_next_hat(4:7) / norm(x_next_hat(4:7)); %Normalise


        end
           
  %------------------------------------------------------------%
  function [x_next_hat, F_k, L_k, proTerm_k] = dyn_update_rect(g, x_k, u_k, t_delta, Rt_imu2rq, reset)
            %F_DYN_2DQUAD_NL Summary of this function goes here
            %   Detailed explanation goes here
            
            % USAGE: 
            
            %
            % INPUTS:
            %     x_k -   state vector for time k: 
            %               x_k= [x; y; z; q_w; q_x; q_y; q_z; x_dot; y_dot; z_dot]
            %     u_k - control input vector for current time step k: 
            %               u_k = [w_x; w_y; w_z; a_x; a_y; a_z] 
            
            %
            % OUTPUTS:
            %     x_next - state est for next time step, based on dynamics
            %     x_cov_k - state covariance for current time step

            %% Initialisations
            % INITIALISATIONS
                    w_k = zeros(6,1); %process noise assumed to be zero-mean gaussian
                           
            %% USE PRECALCULATED MATRICES
                    [x_next_hat, F_k, L_k, proTerm_k] = EKF_3dQuad_10el_funcs.initProcessModelMatrices_rect(x_k, u_k, w_k,t_delta, g);

             %% DERIVE PROCESS MODEL AT START
            % 
            %     if reset == 1
            %         clear persistent;
            %     end
            % 
            %     %Extract variables to match symbolic toolbox output
            %     numerics = [x_k; u_k; w_k; t_delta; g];
            % 
            %     % Persistent variables
            %     persistent F_star;
            %     persistent L_star;
            %     persistent processDE;
            %     persistent xdot_star;
            %     persistent xdot_num;
            %     persistent x_prev;
            %     persistent u_prev;
            %     persistent symbols;
            % 
            %     if isempty(processDE) %if first iteration
            %         % Calculate model
            %         [processDE, F_star, L_star, xdot_star, R_star, symbols] = EKF_3dQuad_10el_funcs.calcProcessModel(g, Rt_imu2rq);
            %     end
            % 
            %     %Use rectangular integration (no previous measurements
            %     %to average)
            %     xdot_num = (double(subs(xdot_star, symbols, numerics)));
            %     x_next_hat = x_k + t_delta * xdot_num;
            %     u_prev = u_k;
            %     x_prev = x_k;
            %     proTerm_k =  xdot_num;
            % 
            %     % process covariance
            %     F_k = double(subs(F_star, symbols, numerics));
            % 
            %     %And L, the input noise covariance: will be used to transform
            %     %covariance in the noise space into the state space. Also
            %     %called the "noise influence matrix"
            %     L_k = double(subs(L_star, symbols, numerics));

            %% Check quaternion term
            %Enforce Smallest angle change
            if dot(x_k(4:7), x_next_hat(4:7)) < 0
                x_next_hat(4:7) = -x_next_hat(4:7);
            end
                       
            x_next_hat(4:7) = x_next_hat(4:7) / norm(x_next_hat(4:7)); %Normalise


        end
           
  %------------------------------------------------------------%
           
  %------------------------------------------------------------%
        function [z_new_hat, H_new] = meas_predict(x_new_hat, Rt_rc2rq, z_k)
       
        % DERIVE MEASUREMENT MODEL
                %Get measurement model
                % [measModel, H_star, symbols_z] = EKF_3dQuad_10el_funcs.calcMeasurementModel(Rt_rc2rq);
                % 
                % %Get residual and H matrix
                % %[r_star, H_quat, symbols_r] = EKF_3dQuad_10el_funcs.calcMeasResidual(measModel);
                % 
                % %set up values to substitute
                % numerics_z = x_new_hat;
                % %numerics_r = [x_new_hat; z_k];
                % 
                % %Get predicted measurement
                % z_new_hat = double(subs(measModel, symbols_z, numerics_z));
                % z_new_hat(4:7) = z_new_hat(4:7)/norm(z_new_hat(4:7)); %normalise quaterniion
                % 
                % %Get predicted measurement covariance
                % %H_new = -double(subs(H_quat, symbols_r, numerics_r));
                % H_new = double(subs(H_star, symbols_z, numerics_z));
    
    
        % USE PREVIOUSLY CALCULATED MATRICES
            
                [z_new_hat, H_new] = EKF_3dQuad_10el_funcs.initMeasModelMatrices(x_new_hat);
    
    
        % GET MEASUREMENT RESIDUAL
                
                %Get measurement residual
                %r_new = double(subs(r_star, symbols_r, numerics_r));
                r_new = z_k - z_new_hat;
                %r_new = [0; 0; 0; 0; 0; 0];

        
        end

  %------------------------------------------------------------%
        
        function [processDE, F_star, L_star, xdot, R_rq2rw, symbols] = calcProcessModel(g, Rt_imu2rq)
        %Function using symbolic toolbox to calculate the matrices involved in
        %the quad process model. 

            %EXtract useful variables
            R_imu2rq =Rt_imu2rq(1:3, 1:3); 
            t_imu2rq =Rt_imu2rq(1:3, 4);
            q_imu2rq = (rotm2quat(R_imu2rq));
            %q_imu2rq = transpose(q_imu2rq); %make column
                    
            %Define symbolic variables

            %general
            dt = sym("dt");
            g = sym("g", [3,1]);

            %state
            p = sym("p", [3,1]);
            v = sym("v", [3,1]);
            q = sym("q", [4,1]);
            x = [p; q; v];

            %"control input"
            u_g = sym("u_g", [3,1]);
            u_a = sym("u_a", [3,1]);
            %u_q = sym("u_q", [4,1]);
            %u = [u_g; u_a; u_q];
            u = [u_g; u_a];

            %process noise
            w_g = sym("w_g", [3,1]);
            w_a = sym("w_a", [3,1]);
            w = [w_g; w_a];

            %measurements (cam)
            p_c  = sym("p_c", [3,1]);
            theta_c = sym("theta_c", [3,1]);
            z = [p_c; theta_c];

            %rotations and conversions
            w_Q = R_imu2rq * (u_g + w_g); %get angular accel in quad frame
            q_u = [0; w_Q]; %turn gyro reading into a quaternion - it is a rate! Don't normalise
            lmo_rq2rw = [q(1) -q(2) -q(3) -q(4) %turn q into left matrix operator for easier math
                        q(2) q(1) -q(4) q(3)
                        q(3) q(4) q(1) -q(2)
                        q(4) -q(3) q(2) q(1)];           
            R_rq2rw = [1 - 2*(q(3)^2 + q(4)^2), 2*(q(2)*q(3) - q(4)*q(1)), 2*(q(2)*q(4) + q(3)*q(1)); % Convert q to rotation matrix
                2*(q(2)*q(3) + q(4)*q(1)), 1 - 2*(q(2)^2 + q(4)^2), 2*(q(3)*q(4) - q(2)*q(1));
                2*(q(2)*q(4) - q(3)*q(1)), 2*(q(3)*q(4) + q(2)*q(1)), 1 - 2*(q(2)^2 + q(3)^2)];
            
            %If IMU is not at drone centre, the rotational component of the
            %acceleration must be removed
            %assuming omega_dot is negligible
            %omega_dot = [0 ; 0; 0];
            %a_rot = cross(omega_dot, t_imu2rq) + cross(u_g, cross(u_g, t_imu2rq));

            %omega_rq = 0.5 * q_u_rw;
            % Calculate process model - somewhat linearising (assuming xdot_prev = xdot)          
            q_dot = 0.5 * lmo_rq2rw * q_u;
            xdot =  [v;
                     q_dot;
                     ((R_rq2rw * R_imu2rq *  (u_a + w_a)) + g)];

            processDE = x + (dt * xdot);
                      
            F_star = jacobian(processDE, x); 

            L_star = jacobian(processDE, u);

            symbols = [x; u; w; dt; g];

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
            %p_z = R_rq2rw * t_rc2rq + p;
           % q_z = q_rc2rw;

            p_z = p;
            q_z = q;
            
        
            %try with quaternions
            measurementModel = [p_z; q_z];

            H_star = jacobian(measurementModel, x);

            symbols = x;

        end
        %------------------------------------------------------------%


        function c = quatProduct(a, b)
            
            c = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*(b(4));
                        a(1)*b(2) + b(1)*a(2) + a(3)*b(4) - b(3)*a(4);
                        a(1)*b(3) + b(1)*a(3) + b(2)*a(4) - a(2)*b(4);
                        a(1)*b(4) + b(1)*a(4) + a(2)*b(3) - b(2)*a(3)];

        end

        %------------------------------------------------------------%
        % function x_c = correctState(x_hat, x_err)
        % 
        %     %Since orientation is a quaternion, it is not updated with a
        %     %naive addition
        % 
        %     x_c_p = x_hat(1:3) + x_err(1:3); %position
        %     x_c_v = x_hat(8:10) + x_err(8:10); %velocity
        % 
        %     x_c_q = EKF_3dQuad_10el_funcs.quatProduct(x_err(4:7), x_hat(4:7)); %quaternion
        % 
        %     x_c = [x_c_p; x_c_q; x_c_v];
        % 
        % end
        %------------------------------------------------------------%
        function [x_new_hat, F_k, L_k, x_dot_k] = initProcessModelMatrices_trap(x_k, u_k, w_k, t_delta, g, reset)

                if reset == 1
                    clear persistent;
                end
                           
                % Persistent variables
                persistent xdot_prev;
                persistent u_prev;

                if isempty(xdot_prev) %if first iteration
                    %Use rectangular integration (no previous measurements)
                    [x_new_hat, F_k, L_k, x_dot_k] = EKF_3dQuad_10el_funcs.initProcessModelMatrices_rect(x_k, u_k, w_k, t_delta, g);
                    xdot_prev =  x_dot_k;
                    u_prev = u_k;
                else
                    %do trapezoidal integration 
                    % Subst in average angular velocity
                    ug_trap = (u_prev(1:3) + u_k(1:3))/2; %get average angular rate
                    u_trap = u_k;
                    u_trap(1:3) = ug_trap;
                    
                    [x_rect, F_k, L_k, xdot_new] = EKF_3dQuad_10el_funcs.initProcessModelMatrices_rect(x_k, u_trap, w_k, t_delta, g);
                    
                    % TRAPEZ-ADISE
                    xdot_trap = (xdot_new + xdot_prev)/2; %average dynamic term
                    xdot_trap(4:7) = xdot_new(4:7); %EXCEPT the quaternion rate - this is captured already
                    
                    x_new_hat = x_k + t_delta * xdot_trap;
                    x_dot_k = xdot_trap;
                    
                    xdot_prev = xdot_trap;
                    u_prev = u_k;
                end
        end

        %------------------------------------------------------------%
        function [x_new_hat, F_k, L_k, x_dot_k] = initProcessModelMatrices_rect(x_k, u_k, w_k, t_delta, g)
            %% Use precalculated model

            % % Variables

            % state
            p1 = x_k(1);
            p2 = x_k(2);
            p3 = x_k(3);
            q1 = x_k(4);
            q2 = x_k(5);
            q3 = x_k(6);
            q4 = x_k(7);
            v1 = x_k(8);
            v2 = x_k(9);
            v3 = x_k(10);

            %inputs
            u_g1 = u_k(1);
            u_g2 = u_k(2);
            u_g3 = u_k(3);
            u_a1 = u_k(4);
            u_a2 = u_k(5);
            u_a3 = u_k(6);

            %noise
            w_g1 = w_k(1);
            w_g2 = w_k(2);
            w_g3 = w_k(3);
            w_a1 = w_k(4);
            w_a2 = w_k(5);
            w_a3 = w_k(6);

            %other
            dt = t_delta;
            g1 = g(1);
            g2 = g(2);
            g3 = g(3);


            % Process model
            x_new_hat = [                                                                                                       p1 + dt*v1;
                                                                                                                                p2 + dt*v2;
                                                                                                                                p3 + dt*v3;
                                                              q1 - dt*((q2*(u_g1 + w_g1))/2 + (q3*(u_g2 + w_g2))/2 + (q4*(u_g3 + w_g3))/2);
                                                              q2 + dt*((q1*(u_g1 + w_g1))/2 - (q4*(u_g2 + w_g2))/2 + (q3*(u_g3 + w_g3))/2);
                                                              q3 + dt*((q1*(u_g2 + w_g2))/2 + (q4*(u_g1 + w_g1))/2 - (q2*(u_g3 + w_g3))/2);
                                                              q4 + dt*((q2*(u_g2 + w_g2))/2 - (q3*(u_g1 + w_g1))/2 + (q1*(u_g3 + w_g3))/2);
                v1 + dt*(g1 - (u_a1 + w_a1)*(2*q3^2 + 2*q4^2 - 1) - (u_a2 + w_a2)*(2*q1*q4 - 2*q2*q3) + (u_a3 + w_a3)*(2*q1*q3 + 2*q2*q4));
                v2 + dt*(g2 - (u_a2 + w_a2)*(2*q2^2 + 2*q4^2 - 1) + (u_a1 + w_a1)*(2*q1*q4 + 2*q2*q3) - (u_a3 + w_a3)*(2*q1*q2 - 2*q3*q4));
                v3 + dt*(g3 - (u_a3 + w_a3)*(2*q2^2 + 2*q3^2 - 1) - (u_a1 + w_a1)*(2*q1*q3 - 2*q2*q4) + (u_a2 + w_a2)*(2*q1*q2 + 2*q3*q4))];
 

            %Jacobians
            F_k =  [[1, 0, 0,                                             0,                                                                  0,                                                                  0,                                                                  0, dt,  0,  0];
                    [0, 1, 0,                                             0,                                                                  0,                                                                  0,                                                                  0,  0, dt,  0];
                    [0, 0, 1,                                             0,                                                                  0,                                                                  0,                                                                  0,  0,  0, dt];
                    [0, 0, 0,                                             1,                                              -dt*(u_g1/2 + w_g1/2),                                              -dt*(u_g2/2 + w_g2/2),                                              -dt*(u_g3/2 + w_g3/2),  0,  0,  0];
                    [0, 0, 0,                          dt*(u_g1/2 + w_g1/2),                                                                  1,                                               dt*(u_g3/2 + w_g3/2),                                              -dt*(u_g2/2 + w_g2/2),  0,  0,  0];
                    [0, 0, 0,                          dt*(u_g2/2 + w_g2/2),                                              -dt*(u_g3/2 + w_g3/2),                                                                  1,                                               dt*(u_g1/2 + w_g1/2),  0,  0,  0];
                    [0, 0, 0,                          dt*(u_g3/2 + w_g3/2),                                               dt*(u_g2/2 + w_g2/2),                                              -dt*(u_g1/2 + w_g1/2),                                                                  1,  0,  0,  0];
                    [0, 0, 0, -dt*(2*q4*(u_a2 + w_a2) - 2*q3*(u_a3 + w_a3)),                       dt*(2*q3*(u_a2 + w_a2) + 2*q4*(u_a3 + w_a3)),  dt*(2*q2*(u_a2 + w_a2) - 4*q3*(u_a1 + w_a1) + 2*q1*(u_a3 + w_a3)), -dt*(2*q1*(u_a2 + w_a2) + 4*q4*(u_a1 + w_a1) - 2*q2*(u_a3 + w_a3)),  1,  0,  0];
                    [0, 0, 0,  dt*(2*q4*(u_a1 + w_a1) - 2*q2*(u_a3 + w_a3)), -dt*(4*q2*(u_a2 + w_a2) - 2*q3*(u_a1 + w_a1) + 2*q1*(u_a3 + w_a3)),                       dt*(2*q2*(u_a1 + w_a1) + 2*q4*(u_a3 + w_a3)),  dt*(2*q1*(u_a1 + w_a1) - 4*q4*(u_a2 + w_a2) + 2*q3*(u_a3 + w_a3)),  0,  1,  0];
                    [0, 0, 0, -dt*(2*q3*(u_a1 + w_a1) - 2*q2*(u_a2 + w_a2)),  dt*(2*q1*(u_a2 + w_a2) + 2*q4*(u_a1 + w_a1) - 4*q2*(u_a3 + w_a3)), -dt*(2*q1*(u_a1 + w_a1) - 2*q4*(u_a2 + w_a2) + 4*q3*(u_a3 + w_a3)),                       dt*(2*q2*(u_a1 + w_a1) + 2*q3*(u_a2 + w_a2)),  0,  0,  1]];
                     
            L_k = [[         0,          0,          0,                         0,                         0,                         0];
                    [         0,          0,          0,                         0,                         0,                         0];
                    [         0,          0,          0,                         0,                         0,                         0];
                    [-(dt*q2)/2, -(dt*q3)/2, -(dt*q4)/2,                         0,                         0,                         0];
                    [ (dt*q1)/2, -(dt*q4)/2,  (dt*q3)/2,                         0,                         0,                         0];
                    [ (dt*q4)/2,  (dt*q1)/2, -(dt*q2)/2,                         0,                         0,                         0];
                    [-(dt*q3)/2,  (dt*q2)/2,  (dt*q1)/2,                         0,                         0,                         0];
                    [         0,          0,          0, -dt*(2*q3^2 + 2*q4^2 - 1),   -dt*(2*q1*q4 - 2*q2*q3),    dt*(2*q1*q3 + 2*q2*q4)];
                    [         0,          0,          0,    dt*(2*q1*q4 + 2*q2*q3), -dt*(2*q2^2 + 2*q4^2 - 1),   -dt*(2*q1*q2 - 2*q3*q4)];
                    [         0,          0,          0,   -dt*(2*q1*q3 - 2*q2*q4),    dt*(2*q1*q2 + 2*q3*q4), -dt*(2*q2^2 + 2*q3^2 - 1)]];

            x_dot_k = [                                                                                                           v1;
                                                                                                                                  v2;
                                                                                                                                  v3;
                                                                - (q2*(u_g1 + w_g1))/2 - (q3*(u_g2 + w_g2))/2 - (q4*(u_g3 + w_g3))/2;
                                                                  (q1*(u_g1 + w_g1))/2 - (q4*(u_g2 + w_g2))/2 + (q3*(u_g3 + w_g3))/2;
                                                                  (q1*(u_g2 + w_g2))/2 + (q4*(u_g1 + w_g1))/2 - (q2*(u_g3 + w_g3))/2;
                                                                  (q2*(u_g2 + w_g2))/2 - (q3*(u_g1 + w_g1))/2 + (q1*(u_g3 + w_g3))/2;
                    g1 - (u_a1 + w_a1)*(2*q3^2 + 2*q4^2 - 1) - (u_a2 + w_a2)*(2*q1*q4 - 2*q2*q3) + (u_a3 + w_a3)*(2*q1*q3 + 2*q2*q4);
                    g2 - (u_a2 + w_a2)*(2*q2^2 + 2*q4^2 - 1) + (u_a1 + w_a1)*(2*q1*q4 + 2*q2*q3) - (u_a3 + w_a3)*(2*q1*q2 - 2*q3*q4);
                    g3 - (u_a3 + w_a3)*(2*q2^2 + 2*q3^2 - 1) - (u_a1 + w_a1)*(2*q1*q3 - 2*q2*q4) + (u_a2 + w_a2)*(2*q1*q2 + 2*q3*q4)];


        end
        %-----------------------------------------------------------%
        
        function [z_new_hat,  H_new] = initMeasModelMatrices(x_new_hat)

             %% Use precalculated matrices
            p1 = x_new_hat(1);
            p2 = x_new_hat(2);
            p3 = x_new_hat(3);
            q1 = x_new_hat(4);
            q2 = x_new_hat(5);
            q3 = x_new_hat(6);
            q4 = x_new_hat(7);

            H_new = [[1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0];
                    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0];
                    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0];
                    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0];
                    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0];
                    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0]];

            z_new_hat =[p1;
                        p2;
                        p3;
                        q1;
                        q2;
                        q3;
                        q4];


            % 
            % z_new_hat = [                             p1;
            %                                              p2;
            %                                              p3;
            %                 (2^(1/2)*q1)/2 - (2^(1/2)*q4)/2;
            %                 (2^(1/2)*q2)/2 + (2^(1/2)*q3)/2;
            %                 (2^(1/2)*q3)/2 - (2^(1/2)*q2)/2;
            %                 (2^(1/2)*q1)/2 + (2^(1/2)*q4)/2];
            % 
            % 
            % H_new = [1, 0, 0,         0,          0,         0,          0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            %             0, 1, 0,         0,          0,         0,          0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            %             0, 0, 1,         0,          0,         0,          0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            %             0, 0, 0, 2^(1/2)/2,          0,         0, -2^(1/2)/2, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            %             0, 0, 0,         0,  2^(1/2)/2, 2^(1/2)/2,          0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            %             0, 0, 0,         0, -2^(1/2)/2, 2^(1/2)/2,          0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            %             0, 0, 0, 2^(1/2)/2,          0,         0,  2^(1/2)/2, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        end
        
        %-----------------------------------------------------------%

        function [ekfResult, P_0, Q, W, t_delta] = initEKF(startTime, x_0)

            %Define P_k - State covariance
            P_0 = diag([0.001, 0.001, 0.001, ...
            0.001, 0.001, 0.001, 0.001, ...
            0.001, 0.001, 0.001]); %Initial, 10 el

            %process noise covariance (noise space)            
            %t_delta_dur = ((u_timeHist(1,2) - u_timeHist(1,1)));
            %t_delta = milliseconds(t_delta_dur) * 0.001;
            t_delta = 0.0623;  %average data rate
            w_g = [(1.037e-03)^2, 0.001^2, 0.00129^2]' / t_delta; %from Allan variance
            w_g = [0.0029^2, 0.005^2, 0.0037^2]';
            w_a = [(5.809e-03)^2, 0.008^2, 0.011^2]' / t_delta; %from Allan variance
            w_a = [0.0128^2, 0.0164^2, 0.0191^2 ]';
            Q = diag([w_g; w_a]);

            Q = diag([0.01, 0.01, 0.01, 0.2, 0.2, 0.2]); %16 el - [w_g, w_acc]
        
            %and measurement covariance
            W =diag([0.1506^2, 0.1506^2, 0.1506^2, 0.01, 0.007897, 0.007897, 0.007897]);  

            
            %initialise EKF output history (do it after first state to get sizes right)
            ekfResult = struct();
            ekfResult.time = createArray(1, 0, 'datetime');
            ekfResult.time = datetime(ekfResult.time, 'Format', 'yyyyMMdd_HHmmss_SSSSSS');
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
            ekfResult.elapsedTime = createArray(1,0, 'double');
            ekfResult.timeSinceLastCorrection = createArray(1,0, 'double');

            %Set first values
            ekfResult.stateEst(:,1) = x_0;
            ekfResult.time(:,1) = datetime(startTime, 'Format', 'yyyyMMdd_HHmmss_SSSSSS');
            ekfResult.elapsedTime(1,1) = 0;
            ekfResult.P(:,:,1) = P_0;
            ekfResult.processTerm(:,1) = zeros(size(x_0));
            ekfResult.measPred(:,1) = zeros(7,1);
            ekfResult.statePred(:,1) = zeros(size(x_0));
            ekfResult.z(:,1) = NaN(7,1);
            ekfResult.input(:,1) = zeros(6,1);
            ekfResult.timeSinceLastCorrection(1,1) = 0;


            
        end
        %-----------------------------------------------------------%


    end
    
end