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

function [yOUT, zOUT, tOUT, te_all, periodicity, varargout] = HybridDynamics(yIN, zIN, p, SMA_L, SMA_R, varargin)
    global SMA_L_database
    global SMA_R_database
    global counter
    global active_leg
    global heat_switch
    global current_Y_R
    global current_Y_L
    global prev_Y_R
    global prev_Y_L
    global right_TD
    global prev_right_TD
    heat_switch = true;
    active_leg = SMA_R.active_leg;
    counter = 1;
    te_index = 1;
    SMA_R_database = [];
    SMA_L_database = [];
    prev_Y_R = 0;
    prev_Y_L = 0;
    current_Y_R = transpose(yIN(2:end-1));
    current_Y_L = ones(size(yIN(2:end-1)));
    right_TD = 0;
    prev_right_TD = 9999;
    te_all = nan(1,100);
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
    index = 1;
    yOUT = NaN(1000, length(yIN));
    zOUT = NaN(1000, length(zIN));;
    tOUT = NaN(1000);
    yOUT(index,:) = yIN;
    zOUT(index,:) = zIN;
    tOUT(index) = tOUT(1:index);
    NoE=0;
    time_step = .01;

    while ~isTerminal
        % Integrate until the next event, maximally for tMAX:
        if isempty(outputIN)
            tspan = [tIN,tMAX];
            disp('This option should be broken')
        else
            try
                tspan = tspan(length(t):end);
                tspan(1) = t(end);
            catch
                tspan = outputIN.getTimeVector(tIN,tMAX);
            end
        end
        % NOTE: even though ode45 is provided with an initial column guess,
        % the results are stored in rows.
        % [t,y,teOUT,yeOUT,ieOUT] = ode_history_dependent(@(t,y) ODE(t,y,SMA_L,SMA_R),tspan,yIN,odeOPTIONS);
        % tspan = [0 5];
        % disp(active_leg)
%           try

            [t,y,teOUT,yeOUT,ieOUT] = ode_history_dependent(@(t,y) ODE(t,y,SMA_L,SMA_R),outputIN.rate, tspan, yIN, @Events, @OutputFcn);
%               try
%                 fprintf('%i\t%f\t%i\n', [ieOUT, teOUT, SMA_R_database.index]);
%               catch
%                 fprintf('%i\t%f\t%i\n',[ieOUT, teOUT, 1]);
%               end
%             if ~isempty(ieOUT)
%                 if ieOUT == 4 && heat_switch && strcmp(active_leg,'right')
%                     heat_switch = false;
%                 end
%             end
            if abs(t(end)-tMAX)<1e-9
                % Time boundary is reached
                yIN = y(end,:)';  % This will be mapped to yOUT below.
                tIN = t(end);
                warning('The integration stops at the time boundary.');
                break;
            end

            if isempty(ieOUT) % || (abs(prev_right_TD - right_TD) < 1e-4)
                % No event occurred. The simulation ran out of time without
                % reaching the terminating event. Map final continuous states
                % (discrete states were not altered) and set time to -1:
                yIN = y(end,:)';  % This will be mapped to yOUT below.
                tIN = -1;
                try
                    periodicity = periodicity*10;
                catch
                    % disp('EMPTY')
                    periodicity = 9999;
                end

                break;
            else
                te_all(te_index) = teOUT;
                te_index = te_index + 1;
                % Handle the discrete change of states at events by calling the
                % jump map (which must be on the MATLAB search path):

                    [yIN, zIN, isTerminal] = JumpMap(yeOUT', zIN, p, ieOUT(end));

                tIN = teOUT(end);

            end

            % disp(abs(prev_right_TD - right_TD))
            if (abs(prev_right_TD - right_TD) < 1e-3) || y(2, end) < 0
                % disp(y(:, end))
                periodicity = 9999;
                break;
            end

            NoE=NoE+1;

            if NoE> 64
                % incorrect footfall sequences, terminate integration;
                break;
            end

            periodicity_R = norm(current_Y_R-prev_Y_R);
            % periodicity_L = norm(current_Y_L-prev_Y_L);
            periodicity = periodicity_R;
%             if periodicity < 1e-3
%                 break
%             end
            if prev_right_TD ~= 9999
                yIN = y(end,:)';  % This will be mapped to yOUT below.
                tIN = -1;

                break;
            end
%          catch
%              break
%          end
    end
    if isa(outputIN, 'SLIP_Model_Graphics_AdvancedPointFeet')
        close(outputIN.video);
    end
    % Map states for return values
    yOUT = yIN;
    zOUT = zIN;
    tOUT = tIN;
    if nargout == 6
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
