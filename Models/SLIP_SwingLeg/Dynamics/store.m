function [] = store(SMA_L, SMA_R)
global SMA_L_database
global SMA_R_database
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
n = 100000;

if isempty(SMA_L_database)
    SMA_L_database.index = 2;
    SMA_R_database.index = 2;
    i = SMA_L_database.index;
    SMA_L_database.eps = NaN(n,1);
    SMA_L_database.T = NaN(n,1);
    SMA_L_database.sigma = NaN(n,1);
    SMA_L_database.MVF = NaN(n,1);
    SMA_L_database.eps_t = NaN(n,1);
    SMA_L_database.E = NaN(n,1);
    % Storing initial conditions
    SMA_L_database.eps(1) = SMA_L.eps_0;
    SMA_L_database.T(1) = SMA_L.T_function(0);
    SMA_L_database.sigma(1) = SMA_L.sigma_0;
    SMA_L_database.MVF(1) = SMA_L.MVF_init;
    SMA_L_database.eps_t(1) = SMA_L.eps_t_0;

    try
        SMA_L_database.eps(i,1) = SMA_L.eps;
        SMA_L_database.T(i,1) = SMA_L.T;
        SMA_L_database.sigma(i,1) = SMA_L.sigma;
        SMA_L_database.MVF(i,1) = SMA_L.MVF;
        SMA_L_database.eps_t(i,1) = SMA_L.eps_t;
        SMA_L_database.E(i,1) = SMA_L.E;
    catch
        0==0;
    end
    
    SMA_R_database.eps = NaN(n,1);
    SMA_R_database.T = NaN(n,1);
    SMA_R_database.sigma = NaN(n,1);
    SMA_R_database.MVF = NaN(n,1);
    SMA_R_database.eps_t = NaN(n,1);
    SMA_R_database.E = NaN(n,1);
    SMA_R_database.t = NaN(n,1);
    
    % Storing initial conditions
    SMA_R_database.eps(1) = SMA_R.eps_0;
    SMA_R_database.T(1) = SMA_R.T_function(0);
    SMA_R_database.sigma(1) = SMA_R.sigma_0;
    SMA_R_database.MVF(1) = SMA_R.MVF_init;
    SMA_R_database.eps_t(1) = SMA_R.eps_t_0;
    SMA_R_database.t(1) = 0;
    try
        SMA_R_database.eps(i,1) = SMA_R.eps;
        SMA_R_database.T(i,1) = SMA_R.T;
        SMA_R_database.sigma(i,1) = SMA_R.sigma;
        SMA_R_database.MVF(i,1) = SMA_R.MVF;
        SMA_R_database.eps_t(i,1) = SMA_R.eps_t;
        SMA_R_database.E(i,1) = SMA_R.E;
    catch
        0==0;
    end
else
        SMA_L_database.index = SMA_L_database.index + 1;
        SMA_R_database.index = SMA_R_database.index + 1;
        i = SMA_L_database.index;
    try 
        SMA_L_database.eps(i,1) = SMA_L.eps;
        SMA_L_database.T(i,1) = SMA_L.T;
        SMA_L_database.sigma(i,1) = SMA_L.sigma;
        SMA_L_database.MVF(i,1) = SMA_L.MVF;
        SMA_L_database.eps_t(i,1) = SMA_L.eps_t;
        SMA_L_database.E(i,1) = SMA_L.E;
    catch
        0==0;
    end
    try
        SMA_R_database.eps(i,1) = SMA_R.eps;
        SMA_R_database.T(i,1) = SMA_R.T;
        SMA_R_database.sigma(i,1) = SMA_R.sigma;
        SMA_R_database.MVF(i,1) = SMA_R.MVF;
        SMA_R_database.eps_t(i,1) = SMA_R.eps_t;
        SMA_R_database.E(i,1) = SMA_R.E;
    catch
        0==0;
    end
end
% fprintf('%i, %i\n', SMA_L_database.index, length(SMA_L_database.eps))
end

