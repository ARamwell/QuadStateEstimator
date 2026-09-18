function [Rt_out]= KneipP3P_Al(X_ABC_W, x_ABC_c_unit)
%Kneip's parametrization of P3P. 
%Let P1 = A, P2 = B, P3 = C. Let cam centre be P. Thus, beta = L(APB);
%alpha = L(BAP). Also, the vectors f1 = fA, f2 = fB, f3 = fC. Nu is camera
%frame, tau is intermediate camera frame. 

    %% Assign some variables
    f1 = x_ABC_c_unit(:,1);
    f2 = x_ABC_c_unit(:,2);
    f3 = x_ABC_c_unit(:,3);
    P1 = X_ABC_W(:,1);
    P2 = X_ABC_W(:,2);
    P3 = X_ABC_W(:,3);
    


    %% Define intermediate frame Tau
    tau_x = f1;
    tau_z = cross(f1, f2);
    tau_z = tau_z/norm(tau_z); %normalise
    tau_y = cross(tau_z, tau_x);
    T_tau_nu = [tau_x tau_y tau_z]; %Rotation matrix
    T_nu_tau = transpose(T_tau_nu);%Transform from cam frame nu to intermed frame tau (no translation)

    %Convert vector f3 into frame tau (needed for phi calc)
    f3_tau = T_nu_tau * f3;

    %% Assert that f3_tau < 0 for theta to be in [0, pi].
    if f3_tau(3,1) > 0

        %Reassign some variables
        f1 = x_ABC_c_unit(:,2);
        f2 = x_ABC_c_unit(:,1);
        f3 = x_ABC_c_unit(:,3);
        P1 = X_ABC_W(:,2);
        P2 = X_ABC_W(:,1);
        P3 = X_ABC_W(:,3);
        % ReDefine intermediate frame Tau
        tau_x = f1;
        tau_z = cross(f1, f2);
        tau_z = tau_z/norm(tau_z); %normalise
        tau_y = cross(tau_z, tau_x);
        T_tau_nu = [tau_x tau_y tau_z]; %Rotation matrix
        T_nu_tau = transpose(T_tau_nu);%Transform from cam frame nu to intermed frame tau (no translation)
        %Convert vector f3 into frame tau (needed for phi calc)
        f3_tau = T_nu_tau * f3;
    end

    %% Define intermediate world frame Eta
    P1P2 = P2-P1;
    P1P3 = P3-P1;
    d12 = norm(P1P2); %More efficient to calculate here
    eta_x = P1P2/d12;
    eta_z = cross(eta_x, P1P3);
    eta_z = eta_z/norm(eta_z); %normalise
    eta_y = cross(eta_z, eta_x);
    T_eta_W = [eta_x eta_y eta_z];
    T_W_eta = transpose(T_eta_W); %Transform from world to eta


    %% Calculate constants

    
    %Convert P3 into frame eta (needed for p1, p2 calc)
    P3_eta = T_W_eta * (P3 - P1);
    
    
    %Calculate beta, i.e., L(APB)
    cosBeta = dot(f1, f2);
    
    % Calculate constants
    b = sqrt((1/(1-cosBeta^2))-1);
    if cosBeta < 0
        b = -b;
    end
    
    phi1 = f3_tau(1)/f3_tau(3); %x/z
    phi2 = f3_tau(2)/f3_tau(3); %y/z
    
    p1 = P3_eta(1);
    p2 = P3_eta(2);
    
    %d12 calculated earlier
    %% Set up quartic terms
    
    a4 = -(phi2^2)*(p2^4) - (phi1^2)*(p2^4) - (p2^4);
    a3 = 2*(p2^3)*d12*b  +  2*(phi2^2)*(p2^3)*d12*b  -  2*phi1*phi2*(p2^3)*d12;
    a2 = -(phi2*p1*p2)^2  -  phi2*(p2*d12*b)^2  -  (phi2*p2*d12)^2  +  (phi2*p2)^2  +  (phi1^2)*(p2^4)  +  2*p1*(p2^2)*d12  +  2*phi1*phi2+p1*(p2^2)*d12*b  -  (phi1*p1*p2)^2  +  2*(phi2^2)*p1*(p2^2)*d12  -  (p2*d12*b)^2  -  2*(p1*p2)^2;
    a1 = 2*(p1^2)*p2*d12*b  +  2*phi1*phi2*(p2^3)*d12  -  2*(phi2^2)*(p2^3)*d12*b  -  2*p1*p2*(d12^2)*b;
    a0 = -2*phi1*phi2*p1*(p2^2)*d12*b  +  (phi2*p2*d12)^2  +  2*(p1^3)*d12  -  (p1*d12)^2  +  (phi2*p1*p2)^2  -  (p1^4)  -  2*(phi2^2)*p1*(p2^2)*d12  +  (phi1*p1*p2)^2  +  (phi2*p2*d12*b)^2;
    
    
    %% Solve quartic to find cosTheta
    q = [a4 a3 a2 a1 a0];
    cosTheta = roots(q);
    cosTheta = real(cosTheta);
    
    %% Subst. to find cotAlpha
    cotAlpha = zeros(4);
    
    num = ((phi1*p1)/phi2) - (d12*b);
    den = (d12 - p1);
    for i = 1:numel(cosTheta)
        cotAlpha(i) = (num + p2*cosTheta(i))/(den + (phi1*p2/phi2)*cosTheta(i));
    end
    
    %% Extract angles
    sinTheta = zeros(4);
    sinAlpha = zeros(4);
    cosAlpha = zeros(4);
    for i = 1:numel(cosTheta)
    
        sinTheta(i) = sqrt(1-(cosTheta(i)^2));
        sinAlpha(i) = sqrt(1/((cotAlpha(i)^2)+1));
        cosAlpha(i) = sqrt(1-(sinAlpha(i)^2));
    
        %Apply angle conditions
        % if f3_tau(3)>0
        %     sinTheta(i) = -sinTheta(i);
        % end
        if cotAlpha(i)<0
            cosAlpha(i) = -cosAlpha(i);
        end
        
    end
    
    %% Subst. angles to find position of camera centre P inside int. frame Eta
    P_eta = zeros(3,4);
    for i = 1:numel(cosTheta)
        P_eta(1,i) = d12 * cosAlpha(i) * ((b*sinAlpha(i))+cosAlpha(i));
        P_eta(2,i) = d12 * sinAlpha(i) * cosTheta(i) * ((b*sinAlpha(i))+cosAlpha(i));
        P_eta(3,i) = d12 * sinAlpha(i) * sinTheta(i) * ((b*sinAlpha(i))+cosAlpha(i));
    end
    
    %% Subst angles to find transformation from Eta to Tau
    T_eta_tau = zeros(3,3,4);
    for i = 1:numel(cosTheta)
        T_eta_tau(:,:,i) = [-cosAlpha(i) -sinAlpha(i)*cosTheta(i) -sinAlpha(i)*sinTheta(i);
                     sinAlpha(i) -cosAlpha(i)*cosTheta(i) -cosAlpha(i)*sinTheta(i);
                     0 -sinTheta(i) cosTheta(i)];
        % T_eta_tau(:,1,i) = [-cosAlpha(i), sinAlpha(i), 0];
        % T_eta_tau(:,2,i) = [-sinAlpha(i)*cosTheta(i), -cosAlpha(i)*cosTheta(i), -sinTheta(i)];
        % T_eta_tau(:,3,i) = [-sinAlpha(i)*sinTheta(i), -cosAlpha(i)*sinTheta(i), cosTheta(i)];
    end
    
    %% Now we have everything we need! 
    %Calculater translation vectors
    t_CW = zeros(3,1);
    R_CW = zeros(3,3,1);
    Rt_CW = zeros(3,4,1);
    Rt_WC = zeros(3,4,1);
    for i = 1:numel(cosTheta)
        t_CW(:,i) = P1 + T_eta_W * P_eta(:,i);
        R_CW(:,:,i) =  T_eta_W * transpose(T_eta_tau(:,:,i)) * T_nu_tau;
        
        Rt_CW(1:3,1:3,i) = R_CW(1:3,1:3,i);
        Rt_CW(1:3,4,i) = t_CW(1:3,i);
    
        %Calc inverse, in case
        Rt_WC(1:3,1:3,i) = transpose(Rt_CW(1:3,1:3,i));
        Rt_WC(1:3,4,i) = - Rt_WC(1:3,1:3,i)*t_CW(1:3,i);

    end

    %% Idea: Calculate W->C first (fewer transposes)
    %Calculate translation vectors
    t_WC = zeros(3,1);
    R_WC = zeros(3,3,1);
    Rt_WC = zeros(3,4,1);
    Rt_WC = zeros(3,4,1);
    for i = 1:numel(cosTheta)
        t_CW(:,i) = P1 + T_eta_W * P_eta(:,i);
        R_WC(:,:,i) = T_tau_nu * T_eta_tau(:,:,i)* T_W_eta;
        t_WC(:,i)= - R_WC(:,:,i) * t_CW(:,i);

        Rt_WC(1:3,1:3,i) = R_WC(1:3,1:3,i);
        Rt_WC(1:3,4,i) = t_WC(1:3,i);
    end

    Rt_out = Rt_WC;
end

