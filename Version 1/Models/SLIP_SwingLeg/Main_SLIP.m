% This is the main script (version 1) that is used to find periodic solutions
% of the bipedal SLIP model with passive swing leg motions. 
% To run the script and conduct numerical continuations, make sure you have 
% the following functions in the searchable paths:

% ************************************
% ************************************
% ..\Dynamics\Definitions:
% ContStateDefinition.m (define continuous states)
% DiscStateDefinition.m (define discrete states)
% SystParamDefinition.m (define system parameters)
% ************************************
% ..\Dynamics:
% FlowMap.m (define continuous dynamics for each phase)
% JumpMap.m (discrete changes at events)
% JumSet.m (event detection function and define terminal event)
% ************************************
% ..\Graphics:
% This folder includes all functions used to play animation of the bipedal 
% model.
% ************************************
% ..\Shared\Synthesis:
% HybridDynamics.m (simulate the system from initial states to the terminal)
% FindPeriodicSolution.m (find periodic motions by solving constraints defined by yOPTIM and zOPTIM)
% FindPeriodicSolutionALP.m (find periodic motions along branches with a fixed distance)
% ************************************
% ..\Shared\Analysis:
% ContactForces.m (compute and draw the ground reaction forces)
% FloquetAnalysis.m (conduct Floquet analysis on periodic motions)

% The following code give you an example of how to find a periodic solution
% for running foward gait and how to find bifurcation points along the
% running in-place branch.

% To find other gaits you have to modify the event detection function in 
% JumSet.m to have the same footfall sequence. 

% Another way to implement this model is provided in folder Version2 with
% which one can use the same script to find all gaits that are shown in the
% paper.

close all
clear all
clc

global SMA_L_database
global SMA_R_database

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
addpath([GaitCreationDir,slash,'Models',slash,'SLIP_SwingLeg',slash,'Inputs;'])
%% (b) Inputs for SMA
IP.phase = 0.;
IP.duty = 1.;
IP.frequency = 1;
IP.amplitude = 324.6427+500;
IP.mass = 10; % kg (used for normalizing)
IP.gravity = 9.80665; % m/s2 (used for normalizing)
[SMA_L, SMA_R] = define_SMA(IP, IP);

%% (b) Basic simulation, find a periodic solution from an initial guess
% Initial values for continuous and discrete states, as well as for the
% system parameters are copied from the basic state definitions: 
yINIT = ContStateDefinition;
zINIT = DiscStateDefinition;
pINIT = SystParamDefinition;
% Define additional options for the simulation:
simOptions.tIN  = 0;  % The simulation will start at t = 0
simOptions.tMAX = 5;  % The simulation will abort when t reaches 5. This prevents an infinite simulation-loop, when the terminal event is missed.

yINIT(contStateIndices.x)       =  0;
yINIT(contStateIndices.dx)      =  2;
yINIT(contStateIndices.y)       =  0.8;
yINIT(contStateIndices.dy)      = -0.4;
yINIT(contStateIndices.alpha)   =  0;
yINIT(contStateIndices.dalpha)  =  0;
yINIT(contStateIndices.phiL)    = -0.5;
yINIT(contStateIndices.dphiL)   =  1.5;
yINIT(contStateIndices.phiR)    =  0.5;
yINIT(contStateIndices.dphiR)   = -1.5;
yINIT(contStateIndices.t)       =  0;

zINIT(discStateIndices.lphase)  =  1; % stance
zINIT(discStateIndices.lcontPt) = -1; % left foot position
zINIT(discStateIndices.rphase)  =  2; % stance
zINIT(discStateIndices.rcontPt) =  0.5; % right foot position

pINIT(systParamIndices.k)       =  20; % Stance leg stiffness
pINIT(systParamIndices.kh)      =  5;  % Swing leg stiffness
disp(pINIT)
% The following arrays define which states and parameters can be altered in
% the root-search. 
yOPTIM(contStateIndices.x)        = 0;  % Always start at x = 0;
yOPTIM(contStateIndices.dx)       = 1;  % The correct forward velocity is found by the root-search
yOPTIM(contStateIndices.y)        = 1;  % The correct vertical height is found by the root-search
yOPTIM(contStateIndices.dy)       = 1;  % The correct vertical velocity is found by the root-search
yOPTIM(contStateIndices.alpha)    = 0;  % Always start at zero pitch angle;
yOPTIM(contStateIndices.dalpha)   = 0;  % Always start at zero pitch velocity;
yOPTIM(contStateIndices.phiL)     = 1;  % The correct left leg angle is found by the root-search
yOPTIM(contStateIndices.dphiL)    = 1;  % The correct left leg velocity is found by the root-search
yOPTIM(contStateIndices.phiR)     = 1;  % The correct right leg angle is found by the root-search
yOPTIM(contStateIndices.dphiR)    = 1;  % The correct left leg velocity is found by the root-search

zOPTIM = zeros(size(zINIT)); 
zOPTIM(discStateIndices.lcontPt)  = 1;  % left foot position is found by the root-search
zOPTIM(discStateIndices.rcontPt)  = 1;  % right foot position is found by the root-search


pOPTIM = zeros(size(pINIT));  % No parameters are altered
pOPTIM(systParamIndices.k)  = 0;
pOPTIM(systParamIndices.kh) = 0;

% Define which states must be periodic. 
yPERIOD(contStateIndices.x)  = 0;  % Forward motion is not periodic;
yPERIOD(contStateIndices.dx) = 1;  % Forward speed must be periodic
yPERIOD(contStateIndices.y)  = 1;  % Hopping height must be periodic
yPERIOD(contStateIndices.dy) = 1;  % Since we always start and stop at apex transit, this is fullfilled automatically
yPERIOD(contStateIndices.alpha)  = 0;  % Always start at y = 1.2;
yPERIOD(contStateIndices.dalpha) = 0;  % Since we always stop at apes transit, we should always start at apex transit
yPERIOD(contStateIndices.phiL)  = 1;  % Always start at y = 1.2;
yPERIOD(contStateIndices.dphiL) = 1;  % Since we always stop at apes transit, we should always start at apex transit
yPERIOD(contStateIndices.phiR)  = 1;  % Always start at y = 1.2;
yPERIOD(contStateIndices.dphiR) = 1;  % Since we always stop at apes transit, we should always start at apex transit

zPERIOD = zeros(size(zINIT)); 
zPERIOD(discStateIndices.lcontPt) = 1;
zPERIOD(discStateIndices.rcontPt) = 1;
% An upper limit for the stride duration is set, such that the simulation
% will be aborted if the terminal state is never reached.  In this case, an
% error message will be created:  
solveOptions.tMAX = 10;
% Call the root-search function.
[yCYC, zCYC, pCYC] =  FindPeriodicSolution(@(yIN, zIN, p, varargin) HybridDynamics(yIN, zIN, p, SMA_L, SMA_R, varargin), yINIT,   zINIT,  pINIT,... 
                                                            yOPTIM,  zOPTIM, pOPTIM,... 
                                                            yPERIOD, zPERIOD, ...
                                                            solveOptions);
                                                    
%% (c) Display the solution:
SMA_L_database = [];
SMA_R_database = [];
pCYC(systParamIndices.k) = NaN; % Stance leg stiffness

figure(1)
simOptions.tMAX = 5; 
recOUTPUT = RecordStateCLASS();
[yOUT, zOUT, tOUT, recOUTPUT] = HybridDynamics(yCYC, zCYC, pCYC, SMA_L, SMA_R, recOUTPUT, simOptions);
% Define which states are plotted:
plotStates = [ contStateIndices.x, contStateIndices.dx,contStateIndices.y, contStateIndices.dy, contStateIndices.phiL, contStateIndices.dphiL,contStateIndices.phiR, contStateIndices.dphiR];
plot(yOUT(:,end),yOUT(:,plotStates))
legend(simRES.continuousStateNames(plotStates));
ContactForces(simRES.continuousStates(:,:),simRES.discreteStates(:,:),pCYC,simRES.t, SMA_L, SMA_R);

% Show animations
SMA_L_database = [];
SMA_R_database = [];
graphOUTPUT = SLIP_Model_Graphics_AdvancedPointFeet(pCYC); % Must be called again with new parameters p, such that the new angle of attack is visualized
[yOUT, zOUT, tOUT] = HybridDynamics(yCYC, zCYC, pCYC, SMA_L, SMA_R, graphOUTPUT, simOptions);
recOUTPUT = RecordStateCLASS();
% [yOUT, zOUT, tOUT, recOUTPUT] = HybridDynamics(yCYC, zCYC, pCYC, SMA_L, SMA_R,recOUTPUT, simOptions);
% simRES = recOUTPUT.retrieve();