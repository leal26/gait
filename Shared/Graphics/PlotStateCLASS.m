% *************************************************************************
%
% classdef PlotStateCLASS(active) < OutputCLASS
%
% This class plots the continuous and discrete states of the system. It's
% an implementation of the interface defined in 'OutputClass'. 
%
% The system is initialized with a Boolean flag that indicates whether it
% should output excitation states 'u' ('active' == true) or not ('active'
% == false). 
%
%
% Properties: - 'continuousPlotting' If this is set to 'false', the
%               plot-output is only updated at the end of the simulation.
% Methods:    - NONE
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
%   See also OUTPUTCLASS, RECORDSTATECLASS, GRAPHIC2SDIMPLELINKCLASS. 
%
classdef PlotStateCLASS < OutputCLASS
    properties (SetAccess = 'private', GetAccess = 'private')
        fig; % The output window for all plots
        axCont;  % The output axis for continuous states
        axDisc;  % The output axis for discrete states
        axExct;  % The output axis for excitation states
        % Data:
        time;            % Time steps
        contStateVector; % The continuous state vector 
        discStateVector; % The discrete state vector
        exctStateVector; % The excitation state vector
        contHandle;      % Plot handle for continuous states
        discHandle;      % Plot handle for discrete states
        exctHandle;      % Plot handle for excitation states
    end
    properties 
        continuousPlotting; 
        % If this flag is set to 'false', the graph is only shown at the end
        % of the simulation.
    end
    methods 
        function obj = PlotStateCLASS(active)
            obj.slowDown = 0; % Run this as fast as possible
            obj.rate = 0.01;  % With a high resolution
            obj.continuousPlotting = true; % Output while simulating
            % Initialize the figure-window and the axes:
            obj.fig = figure();
            set(obj.fig,'Name','State vs. time');
            if active
                n = 3;
                obj.axExct = subplot(n,1,3);
                hold on; grid on; box on;
                title('Excitation States');
                xlabel('time');
                ylabel('exct. state values');
            else
                n = 2;
            end
            obj.axCont = subplot(n,1,1);
            hold on; grid on; box on;
            title('Continuous States');
            xlabel('time');
            ylabel('cont. state values');
            
            obj.axDisc = subplot(n,1,2);
            hold on; grid on; box on;
            title('Discrete States');
            xlabel('time');
            ylabel('disc. state values');
            
            obj.contHandle = [];
            obj.discHandle = [];
            obj.exctHandle = [];
        end
        function obj = update(obj, y, z, t, u)
            if ~isscalar(t)
                 error('GaitCreation:PlotStateCLASS:NoScalar','t must be a scalar. Function can only process one time-step per call');
            end
            obj.time = [obj.time;t];
            obj.contStateVector = [obj.contStateVector,y(:)];
            obj.discStateVector = [obj.discStateVector,z(:)];
            obj.exctStateVector = [obj.exctStateVector,u(:)];
            
            if ~isempty(obj.contStateVector)
                % Check if the plot handle already exists:
                if ~isempty(obj.contHandle)% yes - update
                    for i = 1:length(obj.contHandle)
                        set(obj.contHandle(i),'XData',obj.time,'YData',obj.contStateVector(i,:));
                    end
                else% no - create
                    axes(obj.axCont);
                    obj.contHandle = plot(obj.time,obj.contStateVector);
                    [~, contStateNames] = ContStateDefinition();
                    legend(contStateNames);
                end
            end
            
            if ~isempty(obj.discStateVector)
                % check if the plot handle already exists:
                if ~isempty(obj.discHandle)% yes - update
                    for i = 1:length(obj.discHandle)
                        [x,y] = stairs(obj.time,obj.discStateVector(i,:)');
                        set(obj.discHandle(i),'XData',x,'YData',y);
                    end
                else% no - create
                    axes(obj.axDisc);
                    [x,y] = stairs([obj.time obj.time],[obj.discStateVector obj.discStateVector]');
                    obj.discHandle = plot(x,y);
                    [~, discStateNames] = DiscStateDefinition();
                    legend(discStateNames);
                end
            end
            
            if ~isempty(obj.exctStateVector)
                % check if the plot handle already exists:
                if ~isempty(obj.exctHandle)% yes - update
                    for i = 1:length(obj.exctHandle)
                        set(obj.exctHandle(i),'XData',obj.time,'YData',obj.exctStateVector(i,:));
                    end
                else% no - create
                    axes(obj.axExct);
                    obj.exctHandle = plot(obj.time,obj.exctStateVector');
                    [~, exctStateNames] = ExctStateDefinition();
                    legend(exctStateNames);
                end
            end
            if obj.continuousPlotting
                drawnow 
            end
        end
    end
end
% *************************************************************************
% *************************************************************************