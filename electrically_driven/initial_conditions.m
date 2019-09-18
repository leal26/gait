function [P] = initial_conditions(P, experiment)
% For a given sigma_0 (DV) and P.T_0 (DV) calculate eps_0, MVF_0 and eps_t_0
addpath('../temperature_driven/')

elastic_check = 'N';
integration_scheme = 'I';

n = 2;
P.eps_0 = experiment(1).strain(1,1);
sigma = (experiment(1).stress(1,1) + P.sigma_0)*ones(n,1);
temperature = P.T_0*ones(n,1);

P.eps_t_0 = 0;
P.MVF_0 = 0;

error = 9999;
while error > 1e-2
    [eps,MVF,eps_t,E,MVF_r,eps_t_r ] = Full_Model_stress(temperature, sigma, P, elastic_check, integration_scheme );
    P.eps_t_0 = eps_t(end, end);
    P.MVF_0 = MVF(end, end);
    error = abs(eps(end,end)-P.eps_0)/P.eps_0;
    P.eps_0 = eps(end,end);
end