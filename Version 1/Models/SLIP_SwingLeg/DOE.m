close all
clear all
clc

global SMA_L_database
global SMA_R_database

load('\\coe-fs.engr.tamu.edu\Grads\leal26\Documents\GitHub\gait\periodic_solution.mat')  
y_periodic = simRES.continuousStates(plotStates,:);

%% (a) Setting up working environment
% Define a base directory to make this file easily portable to other computers:

% ************************************
GaitCreationDir = which(mfilename);
GaitCreationDir = erase(GaitCreationDir, [filesep,'Models',filesep,'SLIP_SwingLeg',filesep,'DOE.m']);
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
                                              
%% (c) DOE (frequency, mean, delta, phase):
bounds = [[0,10]; ...
          [375,390]; ...
          [0,20]; ...
          [0,.9]];
parameters_nd = fullfact([8,8,8,8]);
parameters = parameters_nd - 1;
for i=1:4
    parameters(:,i) = bounds(i,1) + (bounds(i,2) - bounds(i,1))*parameters(:,i)/max(parameters(:,i));
end
result_matrix = [parameters, NaN*ones([length(parameters) 4])];

simOptions.tMAX = 5;
pCYC(systParamIndices.k) = NaN; % Stance leg stiffness
IP.mass = 10; % kg (used for normalizing)
IP.gravity = 9.80665; % m/s2 (used for normalizing)
SMA_density = 6450; %kg/m3
for i=1:length(parameters)
    SMA_L_database = [];
    SMA_R_database = [];
    
    IP.frequency = parameters(i,1);
    IP.mean = parameters(i,2);
    IP.amplitude = parameters(i,3);
    IP.phase = parameters(i,4);
    [SMA_L, SMA_R] = define_SMA(IP, IP);  
    
    recOUTPUT = RecordStateCLASS();
    recOUTPUT.rate = 0.001;
    % Checking Austenite constraint
    success = true;
    if SMA_R.T_function(0) >= SMA_R.A_f
        try
            [yOUT, zOUT, tOUT, recOUTPUT] = HybridDynamics(yCYC, zCYC, pCYC, SMA_L, SMA_R, recOUTPUT, simOptions);
            simRES = recOUTPUT.retrieve();
            plotStates = [ contStateIndices.x, contStateIndices.dx,contStateIndices.y, contStateIndices.dy, contStateIndices.phiL, contStateIndices.dphiL,contStateIndices.phiR, contStateIndices.dphiR];

            if min(simRES.continuousStates(contStateIndices.y,:)) > 0
                max_x = max(simRES.continuousStates(contStateIndices.x,:));
                max_y = max(simRES.continuousStates(contStateIndices.y,:));
                max_dy = max(simRES.continuousStates(contStateIndices.dy,:));
                power = calculate_specific_power(SMA_R_database.sigma(1:length(simRES.t)), ...
                                             SMA_R_database.eps(1:length(simRES.t)), ...
                                             SMA_density, recOUTPUT.rate, sqrt(1/IP.gravity));
            else
                success = false;
            end
        catch
            success = false;
        end
    else
        success = false;
    end
    if ~success
        max_x = 9999;
        max_y = 9999;
        max_dy = 9999;
        power = 9999;
    end
    fprintf('%f\t%f\t%f\t%f\t%f\n', i, max_x, max_y, max_dy, power)
    result_matrix(i,end-3) = max_x;
    result_matrix(i,end-2) = max_y;
    result_matrix(i,end-1) = max_dy;
    result_matrix(i,end) = power;
end


% Write to file
fileID = fopen(strcat('DOE.txt'),'w');
fprintf(fileID,'frequency\tmean\tdelta\tphase\tx\ty\tdy\tpower\n');
formatSpec = '%8.3f\t%8.3f\t%8.3f\t%8.3f\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n';
for ii=1:length(parameters)
    fprintf(fileID,formatSpec, result_matrix(ii,:));
end
fclose(fileID);

result_matrix(result_matrix==9999) = NaN;
save('DOE.mat', 'result_matrix', 'parameters_nd')
design_plots(result_matrix, y_periodic)