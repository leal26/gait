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

%% (a) Setting up working environment
% Define a base directory to make this file easily portable to other computers:

% ************************************
GaitCreationDir = which(mfilename);
GaitCreationDir = erase(GaitCreationDir, [filesep,'Models',filesep,'SLIP_SwingLeg',filesep,'Main_SLIP.m']);
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

% x dx y dy alpha dalpha phiL dphiL phiR dphiR t
% yCYC = [0.0000 2.0119 0.9553 0.0000 0.0000 0.0000 0.8620 0.0339 -0.8620 0.0339 0];
% zCYC = [1 0 1 0 0 0 0];
% IP.duty = 1.;   
% 3.909668726968498   0.033978956427332   0.002877539998629   0.004320773548430   0.017146826972944

% brake
% parameters = [387.8571    2.8571    0.5143    0.9000    0.7429];
% actuator
parameters = [390.0000    5.0000    0.7714    0.3857    1.0000];
force = -0.000;

period = 2.7419 ;
right_TD = 0;
IP.frequency =  parameters(5);
IP.mean =    parameters(1)       ;
IP.amplitude = parameters(2)         ;
IP.mass = 10; % kg (used for normalizing)
IP.gravity = 9.80665; % m/s2 (used for normalizing)
IP.active_leg = 'right';
SMA_density = 6450; %kg/m3
IP_L = IP;
IP_L.phase = parameters(3);
IP_R = IP;
IP_R.phase = parameters(4);
% yCYC(2) = parameters(6);
% yCYC(3) = parameters(7);
% yCYC(4) = parameters(8);
% yCYC(7) = - acos(parameters(7));
% yCYC(8) = - parameters(9);
% yCYC(9) = acos(parameters(7));
% yCYC(10) = parameters(9);
[SMA_L, SMA_R] = define_SMA(IP_L, IP_R);

SMA_R.F_external = force;
%% (c) Display the solution:
pCYC(systParamIndices.k) = NaN; % Stance leg stiffness
% yCYC = [0, 1.470855049737456, 0.905287254011324, -0.3840203919504400, 0, 0, -0.452839111566140, 1.141202845250059, 0.438740881829678, -1.168414654087005, 0];
% zCYC = [1.000000000000000, 6.362629437198447, 2.000000000000000, 8.341906161353137, 0, 0, 0];

figure(1)
simOptions.tMAX = 20; 
recOUTPUT = RecordStateCLASS();
recOUTPUT.rate = 0.01;
% figure
% hold on
tic
[yOUT, zOUT, tOUT, te_all, periodicity, recOUTPUT] = HybridDynamics(yCYC, zCYC, pCYC, SMA_L, SMA_R, recOUTPUT, simOptions);
toc
fprintf('Periodicity %f\n', periodicity)
simRES = recOUTPUT.retrieve();
% Define which states are plotted:
plotStates = [ contStateIndices.x, contStateIndices.dx,contStateIndices.y, contStateIndices.dy, contStateIndices.phiL, contStateIndices.dphiL,contStateIndices.phiR, contStateIndices.dphiR];
plot(simRES.t,simRES.continuousStates(plotStates,:))
legend(simRES.continuousStateNames(plotStates));
ContactForces(simRES.continuousStates(:,:),simRES.discreteStates(:,:),pCYC,simRES.t, SMA_L, SMA_R);
sma_plotting(SMA_L, SMA_R, te_all)
phase_space(simRES.continuousStates(plotStates,:))

disp(calculate_specific_power(SMA_R_database.sigma(1:length(SMA_R_database.t)), ...
                             SMA_R_database.eps(1:length(SMA_R_database.t)), ...
                             SMA_density, SMA_R_database.t, 1))
disp(calculate_specific_power(SMA_L_database.sigma(1:length(SMA_R_database.t)), ...
                             SMA_L_database.eps(1:length(SMA_R_database.t)), ...
                             SMA_density, SMA_R_database.t, 1))
% save('mixed')
% Show animations

graphOUTPUT = SLIP_Model_Graphics_AdvancedPointFeet(pCYC); % Must be called again with new parameters p, such that the new angle of attack is visualized
graphOUTPUT.rate = 0.005;
[yOUT, zOUT, tOUT, te_all, periodicity] = HybridDynamics(yCYC, zCYC, pCYC, SMA_L, SMA_R, graphOUTPUT, simOptions);

% recOUTPUT = RecordStateCLASS();
% [yOUT, zOUT, tOUT, recOUTPUT] = HybridDynamics(yCYC, zCYC, pCYC, SMA_L, SMA_R,recOUTPUT, simOptions);
% simRES = recOUTPUT.retrieve();