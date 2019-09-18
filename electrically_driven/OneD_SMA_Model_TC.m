
% Initializing environment
clear all; close all; clc

%--------------------------------------------------------------------------
% INPUTS
%--------------------------------------------------------------------------

% Maximum Number of increments n per loading step
n = 100; 

% Only positive stress?
stress_flag = true;
% INPUT:
% Temperature and time at the start and at the ends of each loading step
% Linear increments strain and temperature loading step assumed
cycles = 20;
frequency = 0.1;

t_inp = [0; 20; ]; %s
for i=1:cycles
    t_inp = [t_inp; [t_inp(end) + 1./frequency/2.,t_inp(end) + 1./frequency]'];
end


eps_min = 0.;
eps_max = .0;
eps_inp = [eps_min; eps_min;];
for i=1:cycles
    eps_inp = [eps_inp; [eps_max,eps_min]'];
end
current_inp = [  0; 0;]; %A
current_min = 0.;
current_max = 2;
for i=1:cycles
    current_inp = [current_inp; [current_max, current_min]'];
end

%% SMA properties
% Young's Modulus for Austenite and Martensite 
P.E_A = 3.7427e+10;
P.E_M = 8.8888e+10;

% Transformation temperatures (M:Martensite, A:
% Austenite), (s:start,f:final)
P.M_s = 300.;
P.M_f = 290.;
P.A_s = 314.6427;
P.A_f = 325.0014;

% Slopes of transformation boundarings into austenite (C_A) and
% martensite (C_M) at Calibration Stress 
P.C_A = 5.1986e+06;
P.C_M = 5.9498e+06;

% Maximum and minimum transformation strain
P.sig_crit = 0;
P.H_min = 0.055;
P.H_sat = 0.0550;
P.k = 4.6849e-09;

% Smoothn hardening parameters 
P.n1 = 0.1919; %0.618;
P.n2 = 0.1823; %0.313;
P.n3 = 0.1623; %0.759;
P.n4 = 0.2188; %0.358;

% Coefficient of thermal expansion
P.alpha_A = 0; 
P.alpha_M = 0;

%% Energy Coefficients
% Mass density
P.rho= 6500; %kg/m^3
% Specific Heat
P.c= 837.36;
% Heat convection coefficient
P.h = 1; % 1 is True and 0 is False
% electrical resistance
P.rho_E =  1e-6;
% Ambient Temperature (Initial Temperature??)
P.T_ambient = 303.15;
P.T_0 = P.T_ambient;
%% Model Geometry
% d: Diameter of considered 1D model
P.d = 0.4e-3;

%% Initial conditions
% initial stress
P.sigma_0 = 0;
% initial MVF
P.MVF_0 = .0;
% initial transformation strain
P.eps_t_0 = 0;

%% Algorithm parameters
% Algorithmic delta for modified smooth hardening function
P.delta=1e-5;
% Calibration Stress
P.sig_cal=200E6;
% Tolerance for change in MVF during implicit iteration
P.MVF_tolerance=1e-8;

%% Generate strain and time states at each increment
% t: time
for i = 1:(size(t_inp,1)-1)
    if i == 1
        t = linspace(t_inp(i), t_inp(i+1), n)';
    else     
        t = [t; linspace(t_inp(i), t_inp(i+1),n)'];
    end
end

% eps: Strain
for i = 1:(size(eps_inp,1)-1)
    if i == 1
        eps = linspace(eps_inp(i), eps_inp(i+1), n)';
    elseif i == 2     
        eps = [eps; linspace(eps_inp(i), eps_inp(i+1),n)'];
    else
        eps = [eps; linspace(eps_inp(i), eps_inp(i+1),n)'];
    end
end

% r: heat source
for i = 1:(size(current_inp,1)-1)
    if i == 1
        current = linspace(current_inp(i), current_inp(i+1), n)';
    else     
        current = [current; linspace(current_inp(i), current_inp(i+1),n)'];
    end
end
% Elastic Prediction Check
% prompt = {'Will the Elastic Prediction Check be Transformation Surface Rate-informed or not (Y/N)?'};
% dlg_title = '1D SMA Model Elastic Prediction Check';
% num_lines = 1;
% defaultans = {'N','hsv'};
% elastic_check = inputdlg(prompt,dlg_title,num_lines,defaultans);
elastic_check = 'N';

[sigma,MVF,T,eps_t,E,MVF_r,eps_t_r, h_convection, pi_t, eps ] = Full_Model_TC( t, eps, current, P, elastic_check, stress_flag);

figure(1)
box on 
plot(eps,sigma/1E6,'b','LineWidth',1.5)
xlabel('Strain')
ylabel('Stress (MPa)')
title('One D SMA Models')
set(gca,'FontName','Times New Roman','fontsize', 20,'linewidth',1.15)
set(gca,'XMinorTick','on','YMinorTick','on')
set(gca,'ticklength',3*get(gca,'ticklength'))

figure(2)
box on 
plot(T,eps,'b','LineWidth',1.5)
xlabel('Temperature (K)')
ylabel('Strain')
title('One D SMA Models T vs. \epsilon')
set(gca,'FontName','Times New Roman','fontsize', 20,'linewidth',1.15)
set(gca,'XMinorTick','on','YMinorTick','on')
set(gca,'ticklength',3*get(gca,'ticklength'))

figure(3)
box on 
plot(T,sigma/1E6,'b','LineWidth',1.5)
xlabel('Temperature (K)')
ylabel('Stress (MPa)')
title('One D SMA Models T vs. \sigma')
set(gca,'FontName','Times New Roman','fontsize', 20,'linewidth',1.15)
set(gca,'XMinorTick','on','YMinorTick','on')
set(gca,'ticklength',3*get(gca,'ticklength'))

figure(4)
box on
hold on
plotyy(t,T,t,MVF);
ylabel('Temperature/MVF')
xlabel('Time (s)')

figure(5)
box on
plot(t,sigma/1E6,'b','LineWidth',1.5)
ylabel('Stress')
xlabel('Time (s)')


figure(6)
box on
plot(t,eps,'b','LineWidth',1.5)
ylabel('Strain')
xlabel('Time (s)')

%% Determine Stress vs. Temperature PHase Diagram
sigma_inp = [0; max(sigma)];

%Generate Stress (sigma) at each increment
for i = 1:(size(sigma_inp,1)-1)
    if i == 1
        sigma_phase = linspace(sigma_inp(i), sigma_inp(i+1), n)';
    else     
        sigma_phase = [sigma_phase; linspace(sigma_inp(i), sigma_inp(i+1),n)'];
    end
end

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

% Arrays of output variables
% T_fwd_0: Temperature array for forward transformation at MVF=0
T_fwd_0 = zeros((size(sigma_phase,1)),1);

% T_fwd_1: Temperature array for forward transformation at MVF=1
T_fwd_1 = zeros((size(sigma_phase,1)),1);

% T_rev_0: Temperature array for reverse transformation at MVF=0
T_rev_0 = zeros((size(sigma_phase,1)),1);

% T_rev_0: Temperature array for reverse transformation at MVF=1
T_rev_1 = zeros((size(sigma_phase,1)),1);

for i = 1:size(sigma_phase,1)
    [T_fwd_0(i,1)]=Forward_Transformation(sigma_phase(i,1),0,P,TP);
    [T_fwd_1(i,1)]=Forward_Transformation(sigma_phase(i,1),1,P,TP);
    [T_rev_0(i,1)]=Reverse_Transformation(sigma_phase(i,1),0,P,TP);
    [T_rev_1(i,1)]=Reverse_Transformation(sigma_phase(i,1),1,P,TP);
end

figure(7)
box on
hold on
plot(T,sigma/1E6,'b','LineWidth',2)
plot(T_fwd_0,sigma_phase/(10^6),T_fwd_1,sigma_phase/(10^6),T_rev_0,sigma_phase/(10^6),T_rev_1,sigma_phase/(10^6),'LineWidth',2)
xlabel('Temperature (K)')
ylabel('Stress (MPa)')
if P.h ==1
    string_initial = 'Natural convection';
else
    string_initial = 'Adiabatic';
end
title([string_initial ', current=' num2str(current_inp(2)) ' Watt/density, frequency=' num2str(frequency) ' Hz'])
set(gca,'FontName','Times New Roman','fontsize', 20,'linewidth',1.15)
set(gca,'XMinorTick','on','YMinorTick','on')
% set(gca,'ticklength',3*get(gca,'ticklength'))
legend('Actuation','\Phi_{fwd, \xi = 0}','\Phi_{fwd, \xi = 1}','\Phi_{rev,  \xi = 0}','\Phi_{rev,  \xi = 1}', 'Location','Southeast')
yL = [min(sigma/1e6) sigma_inp(2)/1e6];
ylim([min(sigma/1e6) max(sigma/1e6)])
xlim([290,405])
% %% Getting work density measurements
% W = eps.*sigma;
% P_i = zeros((size(sigma,1)-1),1);
% % Current transformation strain at calibration stress
% H_cur_cal = H_cursolver(P.sig_cal, P.sig_crit,P.k,P.H_min,P.H_sat);
% 
% % Partial Derivative of H_cur at calibration stress (dH_cur)
% dH_cur=partial_Hcur_sigma(P.sig_cal,P.sig_crit,P.k,P.H_sat,P.H_min);
% 
% % Transformation Parameters (structure: TP)
% rho_delta_s0 = (-2*(P.C_M*P.C_A)*(H_cur_cal+P.sig_cal*dH_cur+P.sig_cal*(1/P.E_M-1/P.E_A)))/(P.C_M+P.C_A);
% for i=1:size(P_i,1)
%      P_i(i) = T(i)*P.alpha*(sigma(i)+sigma(i+1))/2. + ...
%               P.rho*P.c*(T(i) + T(i+1))/2. - ...
%               pi_t(i) + rho_delta_s0*T(i)*(MVF(i) + MVF(i+1))/2.; 
% end
% % P = P.alpha P.c 
% figure(8)
% box on
% %plot(T,W,'b','LineWidth',2)
% plot(T(1:size(T,1)-1),P_i,'b','LineWidth',2)