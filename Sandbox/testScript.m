



[a, b, c] = calcProcessDE;
numerics = [1; 2; 3; 4; 5; 6; 7; 8; 9; 10; 11; 12; 13; 14; 15; 16];
d = double(subs(a, c, numerics));

        function [processModel, F_star, symbols] = calcProcessDE()
        %Function using symbolic toolbox to calculate the matrices involved in
        %the quad process model.
        
            g = [0; 0; 0];
        
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
            q_u = [0; u_g]; %turn gyro reading into a quaternion
            Omega = [q_u(1) -q_u(2) -q_u(3) -q_u(4);
                       q_u(2) q_u(1) q_u(4) -q_u(3);
                       q_u(3) -q_u(4) q_u(1) q_u(2);
                       q_u(4) q_u(3) -q_u(2) q_u(1)];

            %rotation matrix for orientation quaternion
            R = [1 - 2*q(3)^2 - 2*q(4)^2, 2*q(2)*q(3) - 2*q(1)*q(4), 2*q(2)*q(4) + 2*q(1)*q(3);
                 2*q(2)*q(3) + 2*q(1)*q(4), 1 - 2*q(2)^2 - 2*q(4)^2, 2*q(3)*q(4) - 2*q(1)*q(2);
                 2*q(2)*q(4) - 2*q(1)*q(3), 2*q(3)*q(4) + 2*q(1)*q(2), 1 - 2*q(2)^2 - 2*q(3)^2];


            processModel = [v;
                        0.5 * Omega * q;
                        R * u_a + g];


            F_star = jacobian(processModel, x); 

            L_star = jacobian(processModel, u);

            symbols = [x; u];

        end
