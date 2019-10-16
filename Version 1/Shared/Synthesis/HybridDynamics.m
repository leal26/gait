% *************************************************************************
%
% function [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p)
% function [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, exctFcnHndl, s)
%
% -> This MATLAB function simulates the hybrid dynamics associated with the
% functions 'JumpSet', 'JumpMap', and 'FlowMap'.  These functions must be
% available on the MATLAB search path:
%
% - eventValue = JumpSet(y,z,p,exctFcnHndl,s), describes a set of event
%            functions 'e', that trigger the corresponding event if a
%            zero-crossing is detected in positive direction.  
% - [yPLUS, zPLUS, isTerminal] = JumpMap(y ,z, p, exctFcnHndl, s, eventNr), 
%            describes a set of event handlers 'g' that define the
%            instantenous changes of the states during a specific event.
%            If the event is terminal (isTerminal == true), the
%            simulation will be aborted after this event. 
% - yDOT = FlowMap(y, z, p, exctFcnHndl, s), describes the continuous
%            dynamics 'f' of the hybrid system by a first order ODE.
%
% If the system is passive, the result of the rusimulation only depends on
% the initial continuous states 'yIN', the initial discrete states 'zIN',
% and a vector of parameters 'p'.  
%
% An active system depends additionally on an excitation function given by
% 'exctFcnHndl' with a vector of excitation parameters 's', according to:
%
% - u = ExctFcn(y, z, s), which returns the excitation input vector u for
%            every step of the simulation.
%
%
% HybridDynamics returns the continuous and discrete states after the first
% terminal event, together with the time when this happened.
%
% Additionally, the function can be provided with an output object
% 'outputIN' for graphical display or recording of the states, which must
% be derived from the class 'OutputCLASS'. It is updated throughout the
% simulation and returned as 'outputOUT':  
%
% function [yOUT, zOUT, tOUT, outputOUT] = HybridDynamics(yIN, zIN, p, outputIN)
% function [yOUT, zOUT, tOUT, outputOUT] = HybridDynamics(yIN, zIN, p, exctFcnHndl, s, outputIN)
%
% Finally, additional options can be provided by an 'options' struct. The
% following options are provided: 
% - options.tIN  [0]   simulation will start at tIN
% - options.tMAX [inf] simulation will stop at tMAX (to indicate this has
%                      happened, tOUT will be set to -1).
% - options.odeOPTIONS this can be used to change the options for the
%                      MATLAB ODE solver. It must be created with the
%                      'odeset' function. The standard configuration sets: 
%                      'RelTol'=1e-6,'AbsTol'=1e-12,'MaxStep'=0.01.
%
% function [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, options)
% function [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, exctFcnHndl, s, options)
% function [yOUT, zOUT, tOUT, outputOUT] = HybridDynamics(yIN, zIN, p, outputIN, options)
% function [yOUT, zOUT, tOUT, outputOUT] = HybridDynamics(yIN, zIN, p, exctFcnHndl, s, outputIN, options)
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
%   See also FLOWMAP, JUMPMAP, JUMPSET, OUTPUTCLASS.
%

function [yOUT, zOUT, tOUT, varargout] = HybridDynamics(yIN, zIN, p, SMA_L, SMA_R, varargin)
    global SMA_L_database
    global SMA_R_database
    global counter
    counter = 1;
    % *********************************************************************
    % INPUT HANDLING
    %   Check the number of parameters:
    if (nargin>9) || (nargin<5)
        error('GaitCreation:HybridDynamcis:WrongParameterCount', 'Wrong number of input arguments')
    end
    %   Check if we are dealing with an active system:
    if (nargin>5) && (isa(varargin{1},'function_handle'))
        exctFcnHndl = varargin{1};  
        s = varargin{2};
    else
        exctFcnHndl = [];
        s = [];
    end
    %   Check if an output object was provided:
    outputIN = [];
    if nargin>5
        for i = 1:nargin-5
            if isa(varargin{i},'OutputCLASS')
                outputIN = varargin{i};
            end
        end
    end

    % Check if options were provided:
    options = struct([]);
    if (nargin>5) && (isa(varargin{end},'struct')) && ~(nargin == 9 && isa(varargin{3}, 'function_handle'))
        options = varargin{end};
    end
    % Evaluate options:
    if isfield(options,'tMAX')
        tMAX = options.tMAX;
    else
        tMAX = 5; % prevent time going to inf 
    end
    if isfield(options,'tIN')
        tIN = options.tIN;
    else
        tIN = 0;
    end
    % Define basic ode options, which can be overwritten by the user
    % provided values
    odeOPTIONS = odeset('RelTol',1e-6,...
                        'AbsTol',1e-12,...
                        'MaxStep',0.01);
    if isfield(options,'odeOPTIONS') % overwrite with the user provided values
        odeOPTIONS = odeset(odeOPTIONS, options.odeOPTIONS);
    end
    % Set up the options for output and event detection (do this last, as these should never be overwritten by the user):
    odeOPTIONS = odeset(odeOPTIONS,'Events',@Events,'OutputFcn',@OutputFcn);
    % END INPUT HANDLING
    % *********************************************************************
    
    
    
    % *********************************************************************
    % SIMULATE UNTIL TERMINAL EVENT   
                     
    %   Start integration:
    isTerminal = false;
    
    %   Start clock for the timing of the output function
    if ~isempty(outputIN)
        tic
    end
    
    % I suspect there are some zeno effect that keep happening during the
    % integration; so I add a flag to count the total number of events
    % occured during the simulation.
    NoE=0;
    
    while ~isTerminal
        % Integrate until the next event, maximally for tMAX:
        if isempty(outputIN)
            tspan = [tIN,tMAX];
        else
            tspan = outputIN.getTimeVector(tIN,tMAX); 
        end
        % NOTE: even though ode45 is provided with an initial column guess,
        % the results are stored in rows.
        % [t,y,teOUT,yeOUT,ieOUT] = ode_history_dependent(@(t,y) ODE(t,y,SMA_L,SMA_R),tspan,yIN,odeOPTIONS);
        [t,y] = ode_history_dependent(@(t,y) ODE(t,y,SMA_L,SMA_R),2000, tspan,yIN);
        prev_teOUT = [-1 -1 -1 -1 -1 -1];
        index = 2000;
        for i=1:2000
            [teOUT,yeOUT,ieOUT] = Events(0,y(:,i));
            inflections = sign(prev_teOUT).*sign(teOUT);
            for j=1:length(inflections)
                if inflections(j) <= 0
                    index = i;
                    break
                end
            end
            if index ~= 2000
                break
            end
            prev_teOUT = teOUT;
        end
        disp(index)
        disp(size(y))
        t = t(1:index);
        y = y(:, 1:index);
        figure
        plot(t,y(2,:))
        [teOUT,yeOUT,ieOUT] = Events(0,y);
        disp(teOUT)
        BREAK
        if abs(t(end)-tMAX)<1e-9
            % Time boundary is reached
            yIN = y(end,:)';  % This will be mapped to yOUT below.
            tIN = t(end);
            warning('The integration stops at the time boundary.');
            break;  
        end    
        
        if isempty(ieOUT)        
            % No event occurred. The simulation ran out of time without
            % reaching the terminating event. Map final continuous states
            % (discrete states were not altered) and set time to -1:
            yIN = y(end,:)';  % This will be mapped to yOUT below.
            tIN = -1;
            break;    
        else
            % Handle the discrete change of states at events by calling the
            % jump map (which must be on the MATLAB search path): 

                [yIN, zIN, isTerminal] = JumpMap(yeOUT(end,:)', zIN, p, ieOUT(end));   

            tIN = teOUT(end);

        end   
        
        NoE=NoE+1;
        
        if NoE> 12   
            % incorrect footfall sequences, terminate integration;
            break;
        end
            
    end
    
    % Map states for return values
    yOUT = yIN;
    zOUT = zIN;
    tOUT = tIN;
    if nargout == 4
        varargout(1) = {outputIN};
    else
        varargout = {};
    end
    % DONE SIMULATING UNTIL TERMINAL EVENT
    % *********************************************************************
    
    
    
    % *********************************************************************
    % Event Detection   
    function [value_,isterminal_,direction_] = Events(~,y_)
        % Get values of the event function by calling the jump set function
        % (which must be on the MATLAB search path):
        if isempty(exctFcnHndl)
            value_ = JumpSet(y_, zIN, p);
        else
            value_ = JumpSet(y_, zIN,p, exctFcnHndl, s);
        end
        n_events_ = length(value_);
        isterminal_ = ones(n_events_,1); % All events are terminal wrt the ode integration
        direction_  = ones(n_events_,1); % All events require a positive derivative
    end
    % End Event Detection
    % *********************************************************************
    
    % *********************************************************************
    % ODE of the continuous dynamics
    function dydt_ = ODE(t,y_, SMA_L, SMA_R)

        % Get continuous derivatives, by calling the flow map function
        % (which must be on the MATLAB search path):
        if isempty(exctFcnHndl)
            dydt_ = FlowMap(t, y_, zIN, p, SMA_L, SMA_R);
        else
            dydt_ = FlowMap(t, y_, zIN, p, SMA_L, SMA_R, exctFcnHndl, s);
        end
    end
    % End ODE
    % *********************************************************************
    
    % *********************************************************************
    % Call the updating function for the current state
    function status_ = OutputFcn(t_,y_,plot_flag_)
        if ~isempty(outputIN) % if output function exist
            if isempty(plot_flag_)
                for j_ = 1:length(t_)
                    if isempty(exctFcnHndl)
                        u_ = [];
                    else
                        u_ = exctFcnHndl(y_(:,j_), zIN, s);
                    end
                    % call the update function as given in the update object
                    outputIN = update(outputIN, y_(:,j_), zIN, t_(j_), u_);
                    % Wait until actual time equals simulation time (times
                    % factor)
                    while toc<t_(j_)*outputIN.slowDown
                    end
                end
            elseif strcmp(plot_flag_,'init') %First step:
                if isempty(exctFcnHndl)
                    u_ = [];
                else
                    u_ = exctFcnHndl(y_(:), zIN, s);
                end
                % call the update function as given in the update object
                outputIN = update(outputIN, y_(:), zIN, t_(1), u_);
            end
        end
        status_ = 0; % keep integrating
    end
    % End output
    % *********************************************************************
    
end
% *************************************************************************
% *************************************************************************