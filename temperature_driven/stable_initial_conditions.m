function [P] = stable_initial_conditions(P, sigma_0)

if nargin < 2
    sigma_known = false;
else
    sigma_known = true;
end
error = 999;
iter_max = 1000;
iter = 1;

if P.MVF_0 == 1
    if sigma_known == true
        P.sigma_0 = sigma_0;
        H_cur = H_cursolver(P.sigma_0,P.sig_crit,P.k,P.H_min,P.H_sat);
        P.eps_0 = H_cur + sigma_0/P.E_M;
    else
        P.sigma_0 = 0;
        while (error > 1e-6 && iter < iter_max)
            sigma_prev = P.sigma_0;
            H_cur = H_cursolver(P.sigma_0,P.sig_crit,P.k,P.H_min,P.H_sat);
            P.sigma_0 = P.E_M*(P.eps_0-H_cur);
            error = abs(P.sigma_0 - sigma_prev);
        %     disp(P.sigma_0)
        %     disp(sigma_prev)
            iter = iter + 1;
        end
        if iter >= iter_max
            P.eps_0 = 0;
            P.sigma_0 = 0;
        end
    end
elseif P.MVF_0 == 0
    P.sigma_0 = P.E_A*P.eps_0;
end

%% Transformation strain
H_cur = H_cursolver(P.sigma_0,P.sig_crit,P.k,P.H_min,P.H_sat);
if P.MVF_0 == 1
    P.eps_t_0 = H_cur(1,1);
elseif P.MVF_0 == 0
    P.eps_t_0 = 0;
else
    message = 'Not yet implemented';
    error(message)
end