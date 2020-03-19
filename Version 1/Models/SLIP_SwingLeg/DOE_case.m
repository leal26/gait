close all
clear all
clc

global SMA_L_database
global SMA_R_database
global active_leg
global period
global right_TD
global heat_switch

load('\\coe-fs.engr.tamu.edu\Grads\leal26\Documents\GitHub\gait\periodic_solution.mat')  
plotStates = [ contStateIndices.x, contStateIndices.dx,contStateIndices.y, contStateIndices.dy, contStateIndices.phiL, contStateIndices.dphiL,contStateIndices.phiR, contStateIndices.dphiR];
y_periodic = simRES.continuousStates(plotStates,:);

%% (a) Setting up working environment
% Define a base directory to make this file easily portable to other computers:

% ************************************
GaitCreationDir = which(mfilename);
GaitCreationDir = erase(GaitCreationDir, [filesep,'Models',filesep,'SLIP_SwingLeg',filesep,'DOE_case.m']);
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
                                              
%% (c) DOE (mean, delta, phase for left and right legs):
bounds = [[0,1]];
parameters_nd = fullfact([500]);
parameters = parameters_nd - 1;
for i=1:1
    parameters(:,i) = bounds(i,1) + (bounds(i,2) - bounds(i,1))*parameters(:,i)/max(parameters(:,i));
end
result_matrix = [parameters, NaN*ones([length(parameters) 6])];

simOptions.tMAX = 40;
pCYC(systParamIndices.k) = NaN; % Stance leg stiffness
IP.frequency = 1/2.7419;
IP.mass = 10; % kg (used for normalizing)
IP.gravity = 9.80665; % m/s2 (used for normalizing)
IP.active_leg = 'right';
SMA_density = 6450; %kg/m3
% figure(10)
t0 = tic;
hold on
for i=1:length(parameters)
    right_TD = 0;
    IP.mean = 390;
    IP.amplitude = 3;
    IP.phase = parameters(i,1);
    IP.frequency = 1;
    IP_L = IP;
    IP_L.phase = 0;
    IP_R = IP;
    IP_R.phase = parameters(i,1);
    [SMA_L, SMA_R] = define_SMA(IP_L, IP_R);
    SMA_R.F_external = 0;
    recOUTPUT = RecordStateCLASS();
    recOUTPUT.rate = 0.001;
    % Checking Austenite constraint
    success = true;
    if SMA_R.T_function(0) >= SMA_R.A_f
         try
            [yOUT, zOUT, tOUT, te_all, periodicity, recOUTPUT] = HybridDynamics(yCYC, zCYC, pCYC, SMA_L, SMA_R, recOUTPUT, simOptions);
            simRES = recOUTPUT.retrieve();
            plotStates = [ contStateIndices.x, contStateIndices.dx,contStateIndices.y, contStateIndices.dy, contStateIndices.phiL, contStateIndices.dphiL,contStateIndices.phiR, contStateIndices.dphiR];
            plot(simRES.t, simRES.continuousStates(contStateIndices.dx,:), 'DisplayName', num2str(parameters(i,1)))
            if min(simRES.continuousStates(contStateIndices.y,:)) > 0
                max_x = max(simRES.continuousStates(contStateIndices.x,:));
                av_dx = mean(simRES.continuousStates(contStateIndices.dx,:));
                av_dy = mean(simRES.continuousStates(contStateIndices.dy,:));
                power_R = calculate_specific_power(SMA_R_database.sigma(1:length(SMA_R_database.t)), ...
                             SMA_R_database.eps(1:length(SMA_R_database.t)), ...
                             SMA_density, SMA_R_database.t, 1);
                power_L = calculate_specific_power(SMA_L_database.sigma(1:length(SMA_R_database.t)), ...
                             SMA_L_database.eps(1:length(SMA_R_database.t)), ...
                             SMA_density, SMA_R_database.t, 1);
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
        av_dx = 9999;
        av_dy = 9999;
        power_L = 9999;
        power_R = 9999;
        periodicity = 9990;
    end
    % fprintf('%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n', i, max(simRES.t), max_x, av_dx, av_dy, power_L, power_R, periodicity)
    if rem(i,10) == 0
         
        dt = toc(t0);
        dp = i;
        v = dp/dt;
        remaining_p = length(parameters) - i;
        remaining_time = remaining_p/v/3600;
        fprintf('At design %i, remaining time is %f hours\n', i, remaining_time)
    end
    result_matrix(i,end-6) = max(simRES.t);
    result_matrix(i,end-5) = max_x;
    result_matrix(i,end-4) = av_dx;
    result_matrix(i,end-3) = av_dy;
    result_matrix(i,end-2) = power_L;
    result_matrix(i,end-1) = power_R;
    result_matrix(i,end) = periodicity;
end
legend()

% Write to file
fileID = fopen(strcat('DOE_frequencies.txt'),'w');
fprintf(fileID,'mean\tdelta\tphaseL\tphaseR\ttime\tx\tdx\tdy\tpower_L\tpower_R\tperiodicity\n');
formatSpec = '%8.3f\t8.3f\t%8.3f\t%8.3f\t%8.3f\t%8.3f\t%8.3f\t%8.3f\t%8.3f\t%8.3f\t%8.3f\n';
for ii=1:length(parameters)
    fprintf(fileID,formatSpec, result_matrix(ii,:));
end
fclose(fileID);

result_matrix(result_matrix==9999) = NaN;
output = result_matrix(~isnan(result_matrix(:,end)),:);
p = parameters(~isnan(result_matrix(:,end)),:);
save('DOE_30.mat', 'result_matrix', 'parameters_nd', 'output')
design_plots(output, p, y_periodic);

tol = 6e-2;
i_right_actuator = output(:,end-1) > tol;
i_left_actuator = output(:,end-2) > tol;
i_right_brake = output(:,end-1) < -tol;
i_left_brake = output(:,end-2) < -tol;
i_right_strut = ~i_right_actuator & ~i_right_brake;
i_left_strut = ~i_left_actuator & ~i_left_brake;

right_actuator = output(i_right_actuator, :);
left_actuator = output(i_left_actuator, :);
right_brake = output(i_right_brake, :);
left_brake = output(i_left_brake, :);
right_strut = output(i_right_strut, :);
left_strut = output(i_left_strut, :);

target = [-38.799753	-6.666726	];
residual = abs(output(:, end-2:end-1) - target);
[M,I] = min(residual);
disp(p(I,:))