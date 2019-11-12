close all
clear all
clc

global SMA_L_database
global SMA_R_database

load('\\coe-fs.engr.tamu.edu\Grads\leal26\Documents\GitHub\gait\periodic_solution.mat')  
plotStates = [ contStateIndices.x, contStateIndices.dx,contStateIndices.y, contStateIndices.dy, contStateIndices.phiL, contStateIndices.dphiL,contStateIndices.phiR, contStateIndices.dphiR];
y_periodic = simRES.continuousStates(plotStates,:);

%% (a) Setting up working environment
% Define a base directory to make this file easily portable to other computers:

% ************************************
GaitCreationDir = which(mfilename);
GaitCreationDir = erase(GaitCreationDir, [filesep,'Models',filesep,'SLIP_SwingLeg',filesep,'optimization',filesep,'gradient_optimization.m']);
% ************************************
% ************************************
if ~exist(GaitCreationDir,'dir')
    error('Main:GaitCreationDirectorNotExist', 'The specified GaitCreation-directory was not found on your computer.  Please adjust this path to match the installiation on your computer')
end
% ************************************
% ************************************
%
if (isunix)
    slash = '/';
else
    slash = '\';
end
cd([GaitCreationDir, slash, 'Models', slash, 'SLIP_SwingLeg']);
% Reset the MATLAB search path to its default value:
path(pathdef);
% Set the path to include all library functions:
path(path,[GaitCreationDir,slash,'Shared;',...
           GaitCreationDir,slash,'Shared',slash,'Analysis;',...
           GaitCreationDir,slash,'Shared',slash,'Graphics',slash,'SeriesElasticActuation;',...
           GaitCreationDir,slash,'Shared',slash,'Graphics;',...
           GaitCreationDir,slash,'Shared',slash,'Utilities;',...
           GaitCreationDir,slash,'Shared',slash,'Synthesis;']);
% Set the path to include the model specific functions:
% (Every time a different model is processed, it is important to check that
% the path only includes the directories of the current model)
path(path,[GaitCreationDir,slash,'Models',slash,'SLIP_SwingLeg;',...
           GaitCreationDir,slash,'Models',slash,'SLIP_SwingLeg',slash,'Dynamics;',...
           GaitCreationDir,slash,'Models',slash,'SLIP_SwingLeg',slash,'Dynamics',slash,'Definitions;',...
           GaitCreationDir,slash,'Models',slash,'SLIP_SwingLeg',slash,'Graphics;']);
%
% Get the basic state and parameter values, their names, and the
% corresponding index mapping.  By this we can access the vectors by name,
% which keeps the structure very general but allows a clear indexing.
[contStateVec, contStateNames, contStateIndices] = ContStateDefinition();
[discStateVec, discStateNames, discStateIndices] = DiscStateDefinition();
[systParamVec, systParamNames, systParamIndices] = SystParamDefinition();

addpath([GaitCreationDir,slash,'Models',slash,'SLIP_SwingLeg',slash,'SMA_temperature_strain_driven;'])
addpath([GaitCreationDir,slash,'Models',slash,'SLIP_SwingLeg',slash,'phase_diagram;'])
addpath([GaitCreationDir,slash,'Models',slash,'SLIP_SwingLeg',slash,'Inputs;'])
%%
global individuals
global fitnesses
global ind_index
ind_index = 1;

A = [];
b = [];
Aeq = [];
beq = [];

lb = [370, 0, 0, 0, .2];

ub = [400, 10, 1, 1, 6];

% Normalized lower and upper bounds
n_lb = zeros(size(lb));
n_ub = ones(size(ub));

individuals = nan(10000,length(lb));
fitnesses = nan(10000,length(lb));

% Set up parallelization
if max(size(gcp)) == 0 % parallel pool needed
    parpool(16) % create the parallel pool
end

% Define function to be optimized

nonlcon = [];
opts = gaoptimset(...
        'PopulationSize', 200, ...
        'Generations', 200, ...
        'Display', 'iter', ...
        'EliteCount', 4, ...
        'UseParallel', true);
tic
% x = ga(fun, length(lb), A, b, Aeq, beq, n_lb, n_ub, nonlcon, opts);

% n_x0 = (x0 - lb)./(ub-lb);

% Normalized lower and upper bounds
n_lb = zeros(size(lb));
n_ub = ones(size(ub));

% Define function to be optimized
nonlcon = [];
options = optimoptions('fmincon','Display','iter','Algorithm','sqp', 'MaxFunEvals', 1000000, 'PlotFcns',{@optimplotx,...
    @optimplotfval,@optimplotfirstorderopt}, 'UseParallel', true);
forces = linspace(-1e-5, -1e-3) ;
% forces = linspace(-0.000001, -0.001);
n_x0 = [1, .2, .2, .2, .2];
solutions = zeros(10, 5);
objectives = zeros(1,length(forces));
figure(3)
for i=1:length(forces)
    fun = @(x)cost(x, lb, ub,yCYC, zCYC, pCYC, contStateIndices, forces(i), true);
    x = fmincon(fun, n_x0, A, b, Aeq, beq, n_lb, n_ub, nonlcon, options);
    solutions(i,1:5) = n_x0;
    objectives(i) = fun(x);
    n_x0 = x;
    figure(3)
    hold on
    scatter(i,objectives(i),[],'b','filled')
end
toc
disp((ub-lb).*x + lb)
save
% save('opt_results.mat', 'x', 'individuals', 'fitnesses', 'ind_index')