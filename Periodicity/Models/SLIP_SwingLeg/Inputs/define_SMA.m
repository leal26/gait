function [SMA_L, SMA_R] = define_SMA(IP_L, IP_R)
% INPUT:
% If list with three components:
% Temperature and strain at the start and at the ends of each loading step
% Linear increments strain and temperature loading step assumed
% If list with two components:
% Temperature and strain at the start and at the ends of heating
% Linear increments strain and temperature loading step assumed

SMA.area = .25*(3.1415*0.004^2);

% MATERIAL PARAMETERS (Structure: P)
% Young's Modulus for Austenite and Martensite 
SMA.E_A = 3.7427e+10;
SMA.E_M = 8.8888e+10;
% Transformation temperatures (M:Martensite, A:
% Austenite), (s:start,f:final)
SMA.M_s = 363.5013;
SMA.M_f = 297.9735;
SMA.A_s = 324.6427;
SMA.A_f = 385.0014;

% Slopes of transformation boundarings into austenite (C_A) and
% martensite (C_M) at Calibration Stress 
SMA.C_A = 7.1986e+06;
SMA.C_M = 7.9498e+06;

% Maximum and minimum transformation strain
SMA.H_min = 0.0387;
SMA.H_sat = 0.0550;

SMA.k = 4.6849e-09;
SMA.sig_crit = 0;

% Coefficient of thermal expansion
SMA.alpha = 0; %1E-5;

% Smoothn hardening parameters 
% NOTE: smoothness parameters must be 1 for explicit integration scheme
SMA.n1 = 0.1752; %0.618;
SMA.n2 = 0.1789; %0.313;
SMA.n3 = 0.1497; %0.759;
SMA.n4 = 0.2935; %0.358;

% Algorithmic delta for modified smooth hardening function
SMA.delta=1e-5;

% Calibration Stress
SMA.sig_cal=200E6;

% Tolerance for change in MVF during implicit iteration
SMA.MVF_tolerance=1e-8;

% counter for SMA code
SMA.counter = 1;
SMA.MVF_init = 0;
SMA.eps_t_0 = 0;
SMA.sigma_0 = 0;
SMA.eps_0 = 0;
SMA.n = false;
SMA.to_plot = false;

% Define SMA components
SMA_L = SMA;
SMA_R = SMA;

% Define temperature cycles
SMA_L.T_function = @(t) pulse_wave(t, IP_L);
SMA_R.T_function = @(t) pulse_wave(t, IP_R);

% normalizing factor
SMA_L.norm = IP_L.mass*IP_L.gravity;
SMA_R.norm = IP_R.mass*IP_R.gravity;
% T_inp = [T_0; T_final];
% for i = 1:(size(T_inp,1)-1)
    % if i == 1
        % T = linspace(T_inp(i), T_inp(i+1), n)';
    % else     
        % T = [T; linspace(T_inp(i), T_inp(i+1),n)'];
    % end
        
end
