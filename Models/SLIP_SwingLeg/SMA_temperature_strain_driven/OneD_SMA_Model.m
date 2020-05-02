function [P]=OneD_SMA_Model(k, P) %k, T_inp, eps_i,
%  [sigma, MVF, T, eps_t, eps, E, H_cur]=OneD_SMA_Model(k, eps_current, T, MVF_init, eps_t_0, sigma_0, eps_0,n, to_plot) 
% Modified Edwins original code to calculate just for a given T and eps
% Ideal: [sigma, MVF]=OneD_SMA_Model(T_inp, esp_inp)
%Inputs:
%         - k: current iteration of epsilon
%         - T_inp: [T0; Tmax [;T_final]]
%         - eps_i: current SMA strain
%         - n: Maximum Number of increments n per loading step
%         - plot: 'True' or 'False'

global data
% Initial material conditions
MVF_init = P.MVF_init;
eps_t_0 = P.eps_t_0;
sigma_0 = P.sigma_0;
eps_0 = P.eps;
E_0 = P.E_0;
n = P.n;
to_plot = P.to_plot;

% eps: Strain and Temeprature
% if first iteration create list, otherwise load previous and update it
if k == 2
    eps = zeros(2,1);
    eps(1) = eps_0;
    T = zeros(2,1);
    T(1) = P.T_function(0);
else
    old_n = length(data.T);
    eps = [data.eps(1:old_n,1); zeros(1,1)];
    T = [data.T(1:old_n,1); zeros(1,1)];
end
eps(k) = P.eps;
T(k) = P.T;

% Elastic Prediction Check
elastic_check = 'N';

% Integration Scheme
integration_scheme = 'I';

[sigma,MVF,eps_t,E,MVF_r,eps_t_r, H_cur ] = Full_Model( k, T, eps, P, elastic_check, integration_scheme, MVF_init, eps_t_0, sigma_0, E_0, n );

% P.eps = eps;
% P.T =T;
P.sigma = sigma(end);
P.MVF = MVF(end);
P.eps_t = eps_t(end);
P.E = E(end);

if strcmp(to_plot,'True')
    figure()
    box on 
    plot(T,eps,'b','LineWidth',1.5)
    xlabel('Temperature')
    ylabel('Strain')
    title('One D SMA Models')
    set(gca,'FontName','Times New Roman','fontsize', 20,'linewidth',1.15)
    set(gca,'XMinorTick','on','YMinorTick','on')
    set(gca,'ticklength',3*get(gca,'ticklength'))

    figure()
    box on 
    plot(T,sigma,'b','LineWidth',1.5)
    xlabel('Temperature')
    ylabel('Stress (MPa)')
    title('One D SMA Models')
    set(gca,'FontName','Times New Roman','fontsize', 20,'linewidth',1.15)
    set(gca,'XMinorTick','on','YMinorTick','on')
    set(gca,'ticklength',3*get(gca,'ticklength'))
    

    figure()
    box on 
    plot(T, MVF,'b','LineWidth',1.5)
    xlabel('Temperature')
    ylabel('Martensitic volume fraction')
    title('One D SMA Models')
    set(gca,'FontName','Times New Roman','fontsize', 20,'linewidth',1.15)
    set(gca,'XMinorTick','on','YMinorTick','on')
    set(gca,'ticklength',3*get(gca,'ticklength'))

end
