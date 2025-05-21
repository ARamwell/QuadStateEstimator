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
            [x_new_hat, F_k, L_k, processTerm_k] = EKF_3dQuad_10el_funcs.dyn_update(g, x_k, u_k, t_delta, Rt_imu2rq, reset);

                       
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
                [z_new_hat, H_new] = EKF_3dQuad_10el_funcs.meas_predict(x_new_hat, Rt_rc2rq);
            
                %Calculate measurement residual y
                %Enforce quaternion constraints - closest quaternions
                if dot(x_new_hat(4:7), z_new_hat(4:7)) < 0
                    z_new_hat(4:7) = -z_new_hat(4:7);
                end
                if dot(z_new_hat(4:7), z_k(4:7)) < 0
                    z_k(4:7) = -z_k(4:7);
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

   function [x_next_hat, F_k, L_k, proTerm_k] = dyn_update(g, x_k, u_k, t_delta, Rt_imu2rq, reset)
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
        function [z_new_hat, H_new] = meas_predict(x_new_hat, Rt_rc2rq)
        
            % get measurement model every time
            %Get measurement model and associated covariance (symbolic)
            [measModel, H_star, symbols] = EKF_3dQuad_10el_funcs.calcMeasurementModel(Rt_rc2rq);

            %set up values to substitute
            numerics = x_new_hat;

            %Get predicted measurement
            z_new_hat = double(subs(measModel, symbols, numerics));

            %Get predicted measurement covariance
            H_new = double(subs(H_star, symbols, numerics));
            % 
            % % Use precalculated matrices
            % p1 = x_new_hat(1);
            % p2 = x_new_hat(2);
            % p3 = x_new_hat(3);
            % q1 = x_new_hat(4);
            % q2 = x_new_hat(5);
            % q3 = x_new_hat(6);
            % q4 = x_new_hat(7);
            % v1 = x_new_hat(8);
            % v2 = x_new_hat(9);
            % v3 = x_new_hat(10);
            % 
            % z_new_hat = [                             p1;
            %                                          p2;
            %                                          p3;
            %             (2^(1/2)*q1)/2 - (2^(1/2)*q4)/2;
            %             (2^(1/2)*q2)/2 + (2^(1/2)*q3)/2;
            %             (2^(1/2)*q3)/2 - (2^(1/2)*q2)/2;
            %             (2^(1/2)*q1)/2 + (2^(1/2)*q4)/2];
            % 
            % H_new = [1, 0, 0,         0,          0,         0,          0, 0, 0, 0;
            %         0, 1, 0,         0,          0,         0,          0, 0, 0, 0;
            %         0, 0, 1,         0,          0,         0,          0, 0, 0, 0;
            %         0, 0, 0, 2^(1/2)/2,          0,         0, -2^(1/2)/2, 0, 0, 0;
            %         0, 0, 0,         0,  2^(1/2)/2, 2^(1/2)/2,          0, 0, 0, 0;
            %         0, 0, 0,         0, -2^(1/2)/2, 2^(1/2)/2,          0, 0, 0, 0;
            %         0, 0, 0, 2^(1/2)/2,          0,         0,  2^(1/2)/2, 0, 0, 0];



        
        end

  %------------------------------------------------------------%
        
        function [processDE, F_star, L_star, xdot, R_rq2rw, symbols] = calcProcessModel(g, Rt_imu2rq)
        %Function using symbolic toolbox to calculate the matrices involved in
        %the quad process model. 
        
            %g = [0; 0; -9.81];  %world frame
            R_imu2rq =Rt_imu2rq(1:3, 1:3);
            t_imu2rq =Rt_imu2rq(1:3, 4);
            q_imu2rq = (rotm2quat(R_imu2rq));
            %q_imu2rq = transpose(q_imu2rq); %make column
                    
            %Define symbolic variables

            %general
            dt = sym("dt");

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

            %measurements (cam)
            p_c  = sym("p_c", [3,1]);
            theta_c = sym("theta_c", [3,1]);
            z = [p_c; theta_c];

            %rotations
            %get angular accel in quad frame
            w_Q = R_imu2rq * u_g;
            %turn angular accel into a quaternion
            q_u = [0; w_Q]; %turn gyro reading into a quaternion - it is a rate! Don't normalise
          
            lmo_rq2rw = [q(1) -q(2) -q(3) -q(4)
                        q(2) q(1) -q(4) q(3)
                        q(3) q(4) q(1) -q(2)
                        q(4) -q(3) q(2) q(1)];

            % 
            R_rq2rw = [1 - 2*(q(3)^2 + q(4)^2), 2*(q(2)*q(3) - q(4)*q(1)), 2*(q(2)*q(4) + q(3)*q(1));
                2*(q(2)*q(3) + q(4)*q(1)), 1 - 2*(q(2)^2 + q(4)^2), 2*(q(3)*q(4) - q(2)*q(1));
                2*(q(2)*q(4) - q(3)*q(1)), 2*(q(3)*q(4) + q(2)*q(1)), 1 - 2*(q(2)^2 + q(3)^2)];
            % 

            
            % %Manual quaternion rotation
            % q_dot = 0.5 * [-q(2)*w_Q(1)-q(3)*w_Q(2)-q(4)*w_Q(3);
            %              q(1)*w_Q(1)+q(3)*w_Q(3)-q(4)*w_Q(2);
            %              q(1)*w_Q(2)+q(4)*w_Q(1)-q(2)*w_Q(3);
            %              q(1)*w_Q(3)+q(2)*w_Q(2)-q(3)*w_Q(1)]; 
            q_dot = 0.5 * EKF_3dQuad_10el_funcs.quatProduct(q, q_u);
            %q_dot = 0.5 * lmo_rq2rw * q_u;

            %If IMU is not at drone centre, the rotational component of the
            %acceleration must be removed
            %assuming omega_dot is negligible
            %omega_dot = [0 ; 0; 0];
            %a_rot = cross(omega_dot, t_imu2rq) + cross(u_g, cross(u_g, t_imu2rq));

            %omega_rq = 0.5 * q_u_rw;
            
            xdot = [v;
                    q_dot;
                    ((R_rq2rw * R_imu2rq *  u_a) + g)];

            processDE = x + (dt * xdot);
                      

            F_star = jacobian(processDE, x); 

            L_star = jacobian(processDE, u);

            symbols = [x; u; dt];

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

    end
    
end