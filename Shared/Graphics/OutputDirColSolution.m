% *************************************************************************
%
% function outputOUT = OutputDirColSolution(yIN, zIN, p,  exctFcnHndl, s, outputIN, options)
% function outputOUT = OutputDirColSolution(yIN, zIN, p,  outputIN, options)
% 
% This MATLAB function processes a solution obtained by direct collocation
% and sends it to an output object derived from 'OutputCLASS'.  The
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
% Output: - A returned output object 'outputOUT'
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
%   See also OUTPUTCLASS, PLOTSTATECLASS, RECORDSTATECLASS,
%            GRAPHIC2SDIMPLELINKCLASS.  
%
%
function outputOUT = OutputDirColSolution(yIN, zIN, p,  varargin)
    
    % *********************************************************************
    % INPUT HANDLING
    % Check if this is an active or passive system:
    if nargin == 7
        activeSyst = true;
        exctFcnHndl = varargin{1};
        s = varargin{2};
        outputIN = varargin{3};
        options = varargin{4};
    elseif nargin == 5
        activeSyst = false;
        outputIN = varargin{1};
        options = varargin{2};
    else
        error('GaitCreation:OutputDirColSolution:wrongParameterCount', 'Wrong number of input arguments.  Should be 5 for a passive system and 7 for an active system')
    end
    % Evaluate options:
    if isfield(options,'ordEvents')
        ordEvents = options.ordEvents;
        nEvents   = size(ordEvents, 2); 
    else
        error('GaitCreation:FindPeriodicSolution_DIRCOL:noOrdEvents', 'You need to provide the option ''ordEvents'' which determines which events are encountered during the simulation in which order');
    end
    if isfield(options,'nPerInterval')
        nPerInterval = options.nPerInterval;
    else
        error('GaitCreation:FindPeriodicSolution_DIRCOL:noNPerInterval', 'You need to provide an option ''nPerInterval'' which determines how many grid points are used per integration interval');
    end
	if isfield(options,'yGRID') 
        if isfield(options,'tEVENT')
            yGRID = options.yGRID;
            tEVENT = options.tEVENT;
        else
            error('GaitCreation:OutputDirColSolution:noTEVENT', 'You need to provide an option tEVENT, if you provide the option yGRID');
        end
    else
        % Since only the initial conditions are provided, the functions
        % simply calls the HybridDynamics Simulation routine.  This is sort
        % of pointless as it does not take advantage of the DIRCOL
        % solution, and can be done with better control directly by the
        % user. 
        warning('GaitCreation:OutputDirColSolution:noyGRID', 'This functions runs more robust, if yGRID and tEVENT are provided in the options file.');
        if activeSyst
            [~, ~, ~, outputOUT] = HybridDynamics(yIN, zIN, p, exctFcnHndl, s, outputIN);
        else
            [~, ~, ~, outputOUT] = HybridDynamics(yIN, zIN, p, outputIN);
        end
        return    
	end  
    % Check if the number of segments in nPerInterval and yGRID is the same
    nGRIDnoMatch = false;
    for i = 1:nEvents
        if nPerInterval(i) ~= size(yGRID{i},2)
            options.nPerInterval(i) = size(options.yGRID{i},2);
            nGRIDnoMatch = true;
        end
    end
    if nGRIDnoMatch
        warning('GaitCreation:FindPeriodicSolution_DIRCOL:nPerIntervalMissmatch','options.nPerInterval and options.nGRID did not match in segment size.  Optimizer will use the values of options.nGRID');
    end
    % END INPUT HANDLING
    % *********************************************************************
    
    
    % *********************************************************************
    % Process output frame by frame:
    tic
    for i = 1:nEvents  % For all Events
        if i==1
            % Use initial values for the first integration interval
            yPLUS = yIN;
            zPLUS = zIN;
            tPLUS = 0;
        else
            % Apply JumpMap to the last segment of the previous integration
            % interval, to get the start for the next segment.
            [yPLUS, zPLUS] = JumpMap_(yGRID{i-1}(:,end), zPLUS, ordEvents(i-1));
            tPLUS = tEVENT(i-1);
        end
        % Compose this initial value and the provided yGRID into one matrix
        % of continuous states.  Time-steps are evenly spaced throughout
        % the integration interval
        y = [yPLUS, yGRID{i}];
        t = linspace(tPLUS, tEVENT(i), size(y,2));
        for j = 1:size(y,2)
            % Use the appropriate output function:
            if activeSyst
                outputIN = update(outputIN, y(:,j), zPLUS, t(j), exctFcnHndl(y(:,j),zPLUS,s));
            else
                outputIN = update(outputIN, y(:,j), zPLUS, t(j));
            end
            % Wait until actual time equals simulation time (times
            % factor)
            while toc<t(j)*outputIN.slowDown;
            end
        end
    end
    % Process the very last discrete change in states:
    [yPLUS, zPLUS] = JumpMap_(yGRID{end}(:,end), zPLUS, ordEvents(end));
	tPLUS = tEVENT(end);
     % And send it to the output function:
    if activeSyst
        outputIN = update(outputIN, yPLUS, zPLUS, tPLUS, exctFcnHndl(yPLUS,zPLUS,s));
    else
        outputIN = update(outputIN, yPLUS, zPLUS, tPLUS);
    end
    outputOUT = outputIN;
    % *********************************************************************
    
    
    % *********************************************************************
    % Wrapper function to call the jump-map:
    function [yPLUS_, zPLUS_] = JumpMap_(y_, z_, Event_)
        if activeSyst
            [yPLUS_, zPLUS_] = JumpMap(y_, z_, p, exctFcnHndl, s, Event_);
        else
            [yPLUS_, zPLUS_] = JumpMap(y_, z_, p, Event_);
        end
    end
    % *********************************************************************
    
end
% *************************************************************************
% *************************************************************************