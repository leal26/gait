% *************************************************************************
%
% classdef RecordStateCLASS() < OutputCLASS
%
% This class records the continuous and discrete states of the system. It's
% an implementation of the interface defined in OutputClass. 
%
%
% Methods:    - states = retrieve(obj) Returns a struct with all
%               time-series of continuous, discrete, and excitation states.
%
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
%   See also OUTPUTCLASS, PLOTSTATECLASS, GRAPHIC2SDIMPLELINKCLASS. 
%
classdef RecordStateCLASS < OutputCLASS
    properties (SetAccess = 'private', GetAccess = 'private')
        % For data storage:
        time;  % Time vector
        contStateVector; % The continuous state vector 
        discStateVector; % The discrete state vector
        exctStateVector; % The excitation state vector
    end
    methods 
        function obj = RecordStateCLASS()
            obj.slowDown = 0; % Run this as fast as possible
            obj.rate = 0.01;  % With a high resolution
        end
        function obj = update(obj, y, z, t, u)
            if ~isscalar(t)
                 error('GaitCreation:RecordStateCLASS:NoScalar','t must be a scalar. The function can only process one time-step per call');
            end
            obj.time = [obj.time;t];
            obj.contStateVector = [obj.contStateVector,y(:)];
            obj.discStateVector = [obj.discStateVector,z(:)];
            obj.exctStateVector = [obj.exctStateVector,u(:)];
%             disp(t);
        end
        
        % This function (which is not part of the standard interface)
        % returns the current state-trace:
        function states = retrieve(obj)
            states.t = obj.time';
            if ~isempty(obj.contStateVector)
                states.continuousStates = obj.contStateVector;
                [~, states.continuousStateNames] = ContStateDefinition();
            end
            if ~isempty(obj.discStateVector)
                states.discreteStates = obj.discStateVector;
                [~, states.discreteStateNames] = DiscStateDefinition();
            end
            if ~isempty(obj.exctStateVector)
                states.excitationStates = obj.exctStateVector;
                [~, states.excitationStateNames] = ExctStateDefinition();
            end
        end
    end
end
% *************************************************************************
% *************************************************************************