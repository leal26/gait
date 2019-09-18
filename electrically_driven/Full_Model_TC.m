function [ sigma, MVF, T, eps_t, E, MVF_r, eps_t_r, h_convection, pi_t, eps] = Full_Model_TC( t, eps, current, P, elastic_check, stress_flag)
% Function to run the One Dimensional, strain-driven, implicit integration
% scheme
% inputs: time(t), eps(strain), P(material properties), elastic_check(?), T_ambient

% Convert current to heat source (Bhattacharya et al)
r = (P.rho_E/P.rho)*(current.^2)/(pi*(P.d/2)^2)^2;

% Current transformation strain at calibration stress
H_cur_cal = H_cursolver(P.sig_cal, P.sig_crit,P.k,P.H_min,P.H_sat);

% Partial Derivative of H_cur at calibration stress (dH_cur)
dH_cur=partial_Hcur_sigma(P.sig_cal,P.sig_crit,P.k,P.H_sat,P.H_min);

% Transformation Parameters (structure: TP)
TP.rho_delta_s0 = (-2*(P.C_M*P.C_A)*(H_cur_cal+P.sig_cal*dH_cur+P.sig_cal*(1/P.E_M-1/P.E_A)))/(P.C_M+P.C_A);
TP.D = ((P.C_M-P.C_A)*(H_cur_cal+P.sig_cal*dH_cur+P.sig_cal*(1/P.E_M-1/P.E_A)))/((P.C_M+P.C_A)*(H_cur_cal+...
    P.sig_cal*dH_cur));
TP.a1 = TP.rho_delta_s0*(P.M_f-P.M_s);
TP.a2 = TP.rho_delta_s0*(P.A_s-P.A_f);
TP.a3 = -TP.a1/4*(1+1/(P.n1+1)-1/(P.n2+1))+TP.a2/4*(1+1/(P.n3+1)-1/(P.n4+1));
TP.rho_delta_u0 = TP.rho_delta_s0/2*(P.M_s+P.A_f);
TP.Y_0_t = TP.rho_delta_s0/2*(P.M_s-P.A_f)-TP.a3;

%--------------------------------------------------------------------------
% GENERATION OF MEMORY ARRAYS
%--------------------------------------------------------------------------

% Arrays of output variables
% T: Temperature
T = zeros((size(t,1)),1);
T(1,1)=P.T_0;
% H_cur: Current maximum transformational strain
H_cur = zeros((size(t,1)),1);
% eps_t: Transformational Strain
eps_t = zeros((size(t,1)),1);
% sig: Stress
sigma = zeros((size(t,1)),1);
% MVF: Martensitic Volume Fraction
MVF = zeros((size(t,1)),1);
% E: Youngs Modulus
E = zeros((size(t,1)),1);
% eps_t_r: transformational strain at transformation reversal
eps_t_r = zeros((size(t,1)),1);
% MVF_r: Martensic Strain at transformation reversal
MVF_r = zeros((size(t,1)),1);
%Phi_fwd: Forward transformation surface
Phi_fwd = zeros((size(t,1)),1);
%Phi_rev: Reverse transformation surface
Phi_rev = zeros((size(t,1)),1);
%h :  heat convection coefficient
h_convection = zeros((size(t,1)),1);
%pi_t: Thermal force
pi_t = zeros((size(t,1)),1);

% Initialize outputs
H_cur(1,1) = P.H_sat;
eps_t(1,1) = P.eps_t_0;
sigma(1,1) = P.sigma_0;
MVF(1,1) = P.MVF_0;
E(1,1)=1/(1/P.E_A+MVF(1,1)*(1/P.E_M-P.E_A));

% Array for number of iterations required for each load step
increments = zeros((size(t,1)),1);
for i = 2:size(t,1)
    increments(i,1)=0;
    % Initialize Output Variables
    MVF(i,1)=MVF(i-1,1);
    eps_t(i,1)=eps_t(i-1,1);
    E(i,1)=E(i-1,1);
    MVF_r(i,1)=MVF_r(i-1,1);
    eps_t_r(i,1)=eps_t_r(i-1,1);
    
    % Calculate thermal expansion
    P.alpha = MVF(i,1)*P.alpha_M + (1-MVF(i,1))*P.alpha_A;
    
    % Calculate heat source
    r_i = r(i,1);
    P.r = r_i;
    % Elastic prediction and transformation Check
    % Determine user input for transformation check
    if strcmpi(elastic_check,'No') || strcmpi(elastic_check,'N')
        % Non-transformation surface rate-informed selected
        [ sigma(i,1), T(i,1), eps_t(i,1), MVF(i,1), H_cur(i,1), Phi_fwd(i,1), Phi_rev(i,1), h_convection(i,1), eps(i,1), chck ] = ...
            Elastic_Transformation_check_TC( P, TP, eps(i,1), eps(i-1,1), t(i,1), t(i-1,1), T(i-1,1), T(1,1), sigma(i-1,1), Phi_fwd(i-1,1), Phi_rev(i-1,1), E(i,1), MVF(i,1), eps_t(i,1), eps_t_r(i,1), MVF_r(i,1), r_i, stress_flag);
    elseif strcmpi(elastic_check,'Yes') || strcmpi(elastic_check,'Y')
        % Transformation surface rate-informed selected
        [ sigma(i,1), T(i,1), eps_t(i,1), MVF(i,1), H_cur(i,1), Phi_fwd(i,1), Phi_rev(i,1), h_convection(i,1), chck ] = ...
            Elastic_Transformation_check_RI_TC(P, TP, eps(i,1), eps(i-1,1), t(i,1), t(i-1,1), T(i-1,1), T(1,1), sigma(i-1,1), Phi_fwd(i-1,1), Phi_rev(i-1,1), E(i,1), MVF(i,1), eps_t(i,1), eps_t_r(i,1), MVF_r(i,1), r_i );
        
    else
        % Display error if neither Yes or No are selected
        disp('Please input "No" or "Yes" for Elastic Prediction: Transformation Surface Rate-informed');
        break
    end
    
    % Use inplicit integration scheme considering thermomechanical coupling
    % for transformation correction
    if chck ~= 0
        [ MVF(i,1), T(i,1), eps_t(i,1), E(i,1), MVF_r(i,1), eps_t_r(i,1), sigma(i,1), Phi_fwd(i,1), Phi_rev(i,1), pi_t(i,1) ] = ...
            Implicit_Transformation_Correction_TC(P, TP, chck,...
            MVF(i,1),eps_t(i,1),E(i,1),MVF_r(i,1),eps_t_r(i,1),sigma(i,1), H_cur(i,1),eps(i,1), T(i,1), T(1,1), Phi_fwd(i,1), Phi_rev(i,1) );
    end
end
end

