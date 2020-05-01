% *************************************************************************
%
% function optionsOUT = ResampleDircolGrid(yIN, zIN, p,  exctFcnHndl, s, n, options)
% function optionsOUT = ResampleDircolGrid(yIN, zIN, p,  n, options)
% 
% This MATLAB function processes a solution obtained by direct collocation
% and resamples it such that it contains a total of 'n' grid points.  The
% solution is characterized by an options struct, initial conditions, and
% associated parameters.  
% 
%
% Input:  - A vector of initial continuous states 'yIN' 
%         - A vector of initial discrete states 'zIN' 
%         - A vector of model system parameters 'p'
%         OPTIONAL:
%           - An excitation function u = ExcitationFunction(y, z, s) that 
%             describes the position of the drive side of the series 
%             elastic actuator. If it is not provided the initial guess
%             from the definition function (EXCTSTATEDEFINITION) is used.
%           - A vector of excitation parameters 's';
%         - An output object 'outputIN', derived from the class OutputCLASS
%           for graphical display or recording of the states. It is updated
%           throughout the simulation and returned as 'outputOUT'    
%         - The number of grid points 'n' in the processed solution.
%         - Additional options must be provided with the 'options' struct: 
%             * options.ordEvents A vector of all events that are 
%                                 encountered in this order during the 
%                                 simulation 'ordEvents' 
%             * options.nPerInterval A vector that indicates the number of
%                                    grid-points used in every single
%                                    integration interval.
%             * options.yGRID An initial grid for the continuous states.
%             * options.tEVENT A vector of points in time at which the
%                              events happen.
%
% Output: - An options struct 'optionsOUT' with the new, resampled grid
%
% Created by C. David Remy on 03/14/2011
% MATLAB 2010a
%
% Documentation:
%  'A MATLAB Framework For Gait Creation', 2011, C. David Remy (1), Keith
%  Buffinton (2), and Roland Siegwart (1),  International Conference on
%  Intelligent Robots and Systems, September 25-30, San Francisco, USA 
%
% (1) Autonomous Systems Lab, Institute of Robotics and Intelligent Systems, 
%     Swiss Federal Institute of Technology (ETHZ) 
%     Tannenstr. 3 / CLA-E-32.1
%     8092 Zurich, Switzerland  
%     cremy@ethz.ch; rsiegwart@ethz.ch
%
% (2) Department of Mechanical Engineering, 
%     Bucknell University
%     701 Moore Avenue
%     Lewisburg, PA-17837, USA
%     buffintk@bucknell.edu
%
%   See also FINDPERIODICSOLUTION_DIRCOL, OUTPUTDIRCOLSOLUTION.  
%
%
function optionsOUT = ResampleDircolGrid(yIN, zIN, p,  varargin)
    
    % *********************************************************************
    % INPUT HANDLING
    % Check if this is an active or passive system:
    if nargin == 7
        activeSyst = true;
        exctFcnHndl = varargin{1};
        s = varargin{2};
        n = varargin{3};
        options = varargin{4};
    elseif nargin == 5
        activeSyst = false;
        n = varargin{1};
        options = varargin{2};
    else
        error('GaitCreation:ResampleDircolGrid:wrongParameterCount', 'Wrong number of input arguments.  Should be 5 for a passive system and 7 for an active system')
    end
    % The record state class is used to retrieve a full state-trace of the
    % solution.  It will also check fi the provided 'options' struct is
    % valid:
    recOUTPUT = RecordStateCLASS();
    if activeSyst
        recOUTPUT = OutputDirColSolution(yIN, zIN, p,  exctFcnHndl, s, recOUTPUT, options);
    else
        recOUTPUT = OutputDirColSolution(yIN, zIN, p, recOUTPUT, options);
    end
    simRES = recOUTPUT.retrieve();
    nEvents = length(options.tEVENT);
    % END INPUT HANDLING
    % *********************************************************************
    
    
    % *********************************************************************
    optionsOUT = options;
    % Compute the number of grid points in each integration interval:
    optionsOUT.nPerInterval = max(1,ceil([options.tEVENT(1);diff(options.tEVENT)]*n/options.tEVENT(end)));
    % Resample the state trace in each integration interval:
    for i = 1:nEvents
        if i == 1
            tStart = 0;
        else
            tStart = options.tEVENT(i-1);
        end
        tEnd =  options.tEVENT(i);
        % Define new grid over time:
        tNew = linspace(tStart, tEnd, optionsOUT.nPerInterval(i)+1);
        % resample
        optionsOUT.yGRID{i} = interp1(simRES.t+linspace(0,1e-5,length(simRES.t)), simRES.continuousStates', tNew(2:end),'cubic')';
    end
end
% *************************************************************************
% *************************************************************************