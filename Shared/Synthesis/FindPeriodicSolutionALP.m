% *************************************************************************
%
% function [yCYC, zCYC, pCYC, sCYC, costValue] =  ...
%     FindPeriodicSolution(hybrDynHndl, yINIT,   zINIT,  pINIT,  exctFcnHndl, sINIT, ... 
%                                       yOPTIM,  zOPTIM, pOPTIM,              sOPTIM, ... 
%                                       yPERIOD, zPERIOD, ...
%                                       yCOST,   zCOST,  pCOST,               sCOST, ...
%                                       options)    
%
% function [yCYC, zCYC, pCYC, costValue] =  ...
%     FindPeriodicSolution(hybrDynHndl, yINIT,   zINIT,  pINIT,... 
%                                       yOPTIM,  zOPTIM, pOPTIM,... 
%                                       yPERIOD, zPERIOD, ...
%                                       yCOST,   zCOST,  pCOST, ...
%                                       options)
%
% function [yCYC, zCYC, pCYC, sCYC] =  ...
%     FindPeriodicSolution(hybrDynHndl, yINIT,   zINIT,  pINIT,  exctFcnHndl, sINIT, ...
%                                       yOPTIM,  zOPTIM, pOPTIM,              sOPTIM,...
%                                       yPERIOD, zPERIOD, ...
%                                       options)
%
% function [yCYC, zCYC, pCYC] =  ...
%     FindPeriodicSolution(hybrDynHndl, yINIT,   zINIT,  pINIT,... 
%                                       yOPTIM,  zOPTIM, pOPTIM,... 
%                                       yPERIOD, zPERIOD, ...
%                                       options)
%
% This function returns a periodic solution for the model described by
% 'hybrDynHndl'. It starts a constrained optimization algorithm that
% minimizes a costfunction composed by the sum of the final values of
% designated states and parameters while maintaining a periodic solution
% in other states.  
% The states and parameters provided by the calling function ('yINIT',
% 'zINIT, ...) are either left unchanged, or serve as the initial guess
% for optimization.   
% Which states and parameters are optimized, must be periodic, or
% contribute to the cost function can be set via the flags '*OPTIM', 
% '*PERIOD', '*COST', ...
% Variables that are not periodic within a step, do not contribute to the
% cost function, and should have a fixed starting value (like the
% horizontal position) should be excluded.   
%
% Input:  - A one stride gait model with the following interface:
%           [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, exctFcnHndl, s, options)
%           [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, options)
%           This function maps a set of initial states to a set of final
%           states, using the given parameters. The user must support the
%           function handle 'hybrDynHndl' to this model.
%         - An initial guess for the continuous states 'yINIT'
%         - An initial guess for the discrete states 'zINIT'
%         - An initial guess for the system parameters 'pINIT'
%         OPTIONAL:
%           - An excitation function u = ExcitationFunction(y, z, s) that 
%             describes the position of the drive side of the series 
%             elastic actuator. If it is not provided the initial guess
%             from the definition function (EXCTSTATEDEFINITION) is used.
%           - A vector of excitation parameters 'sINIT';
%         - For each of these initial guess, the following sets of flags
%           that indicate:  
%           ... *OPTIM: The corresponding initial guess is being
%                       optimized [flag = 1] or not [flag = 0].
%           ... *PERIOD: The corresponding state is made periodic [flag = 1]
%                        or not [flag = 0].
%           OPTIONAL
%           ... *COST: The corresponding state or parameter contributes to
%                      the cost function [flag>0] in this case, the flag
%                      also indicates a weight that is used when the
%                      individual cost-terms are composed. If a
%                      state/parameter should not contribute to the cost
%                      function, the corresponding flag should be set to 0.
%         - Additional options are provided with the 'options' struct: 
%            * options.tMAX A maximal simulation time, after which the
%                           integration is aborted (this MUST be provided).
%            * options.numOPTS A set of parameters for the numerical
%                              optimization(created by the 'optimset'
%                              function).  This will override the standard
%                              values set below.  It can, for example, be
%                              used to specify a number for maximal
%                              function evaluations.
%
% Output: - Initial continuous states 'yCYC', that result in an optimal
%           periodic motion 
%         - Initial discrete states 'zCYC', that result in an optimal
%           periodic motion
%         - System parameters 'pCYC', that result in an optimal
%           periodic motion
%         OPTIONAL:
%           - Excitation parameters 'sCYC', that result in an optimal
%             periodic motion
%           - The final value of the overall cost-function 'costValue'
%
% Created by C. David Remy on 03/14/2011
% MATLAB 2010a
% (uses the optimization toolbox)
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
%   See also HYBRIDDYNAMICS, FLOQUETANALYSIS.
%
function [yCYC, zCYC, pCYC, varargout] = FindPeriodicSolutionALP(hybrDynHndl, yINIT,   zINIT,  pINIT, varargin)
    
    % *********************************************************************
    % Input handling
    if ((nargin ~= 12) && (nargin ~= 13) && (nargin ~= 17))
        error('GaitCreation:FindPeriodicSolution:wrongParameterCount', 'Wrong number of input arguments.  Should be 10 for a passive system with no cost function, 13 for a passive system with cost function or an active system with no cost function, and 17 for an active system with cost function')
    end
    % Create some detailed information for the screen:
    disp('Looking for a periodic gait...');
    if isa(varargin{1},'function_handle')
        disp(' ... for an active system');
        activeSyst  = true;
        exctFcnHndl = varargin{01};
        sINIT       = varargin{02};
        yOPTIM      = varargin{03};
        zOPTIM      = varargin{04};
        pOPTIM      = varargin{05};
        sOPTIM      = varargin{06};
        yPERIOD     = varargin{07};
        zPERIOD     = varargin{08};
    else
        disp(' ... for a passive system');
        activeSyst  = false;
        yOPTIM      = varargin{01};
        zOPTIM      = varargin{02};
        pOPTIM      = varargin{03};
        yPERIOD     = varargin{04};
        zPERIOD     = varargin{05};
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        yCYC_OLD    = varargin{06};
        ALPstep     = varargin{07};
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
        sINIT       = [];
        sOPTIM      = [];
    end
    if (nargin == 17 && activeSyst)  || (nargin == 13 && ~activeSyst) 
        disp(' ... with a cost function');
        costFct = true;
        if activeSyst % active system with cost function
            yCOST       = varargin{09};
            zCOST       = varargin{10};
            pCOST       = varargin{11};
            sCOST       = varargin{12};
            options     = varargin{13};  
        else
            yCOST       = varargin{06};
            zCOST       = varargin{07};
            pCOST       = varargin{08};
            sCOST       = [];
            options     = varargin{09}; 
        end
    else
        disp(' ... without a cost function');
        costFct = false;
        if activeSyst % active system without cost function
            options     = varargin{09}; 
        else
            options     = varargin{08};
        end
    end
    % Evaluate options:
    if isfield(options,'tMAX')
        tMAX = options.tMAX;
    else
        error('GaitCreation:FindPeriodicSolution:noTMAX', 'You need to provide an option tMAX, which will be used to prevent infinite simulation loops');
    end
    % Define basic options for the numerical optimization/root search,
    % which can be overwritten by the user:
    if costFct % Systems with a cost function
        numOPTS = optimset('Algorithm','active-set',...
                           'Display','iter',...
                           'MaxFunEvals',200,...
                           'MaxIter',500,...
                           'TolX',1e-8,...
                           'TolFun',1e-8,...
                           'TolCon',1e-6,...
                           'LargeScale','off',...
                           'DiffMinChange',1e-12,...
                           'DiffMaxChange',1e-3,...
                           'RelLineSrchBnd',1e-3,...
                           'RelLineSrchBndDuration',1e8);
    else % Systems without a cost function
        numOPTS = optimset('Algorithm','levenberg-marquardt',... 
                           'Display','iter',...
                           'MaxFunEvals',600,...
                           'MaxIter',500,...
                           'TolFun',1e-12,...
                           'TolX',1e-12,...
                           'DiffMinChange',1e-12,...
                           'DiffMaxChange',1e-3);
    end
    if isfield(options,'numOPTS') % overwrite with the user provided values
        numOPTS = optimset(numOPTS, options.numOPTS);
    end
    
    % END INPUT HANDLING
    % *********************************************************************
    
    
    
    % *********************************************************************
    % Optimization/Rootsearch:
    % Set up the first guess that contains the initial states that are
    % subject to change and the free parameters: 
    xINIT = MapYZPStoX(yINIT, zINIT, pINIT, sINIT);
    % Run simulation once to initialize the past values:
    yIN_OLD = [];
    zIN_OLD = [];
    p_OLD = [];
    s_OLD = [];
    yOUT_OLD = [];
    zOUT_OLD = [];
    

    RunSimulation(xINIT);
    
    % Apply the appropriate algorithm:
    if ~costFct
        % Only finding a periodic solution:
        disp('Running fsolve...');
        % Two different options are implemented for the root search. A
        [xCYC, constViol] = fsolve(@RootTransferFunction, xINIT, numOPTS);
        [yCYC, zCYC, pCYC, sCYC] = MapXtoYZPS(xCYC);
        if activeSyst
            varargout = {sCYC};
        else
            res = constViol'*constViol;
            varargout = {res};
        end
        disp([' Residual = ', num2str(res)]);
        disp('...done with root search'); 
    else
        % Minimization of cost-function:
        disp('Running fmincon...');
        [xCYC, costValue, ~, output] = fmincon(@CostFunction, xINIT, [], [], [], [], [], [], @ConstrainFunction, numOPTS);
        [yCYC, zCYC, pCYC, sCYC] = MapXtoYZPS(xCYC);
        if activeSyst
            varargout = {sCYC, costValue};
        else
            varargout = {costValue};
        end
        disp([' Residual  = ', num2str(output.constrviolation)]);
        disp([' Costvalue = ', num2str(costValue)]);
        disp('...done with optimization'); 
    end
    % *********************************************************************
             
    
    % *********************************************************************
    % Local functions:
    function [y_, z_, p_, s_] = MapXtoYZPS(x_)
        % In this function, the optimization vector x_ is mapped into
        % individual states and parameters. y_, z_, p_, and s_ are
        % one-dimensional column vectors. 

        % Copy the initial values, as most of them remain unchanged.
        y_ = yINIT;
        z_ = zINIT;
        p_ = pINIT;
        s_ = sINIT;
        % Change the ones that were optimized:
        y_(yOPTIM == 1) = x_(1:nnz(yOPTIM == 1));
        z_(zOPTIM == 1) = x_(nnz(yOPTIM == 1) + 1:nnz(yOPTIM == 1) + nnz(zOPTIM == 1));
        p_(pOPTIM == 1) = x_(nnz(yOPTIM == 1) + nnz(zOPTIM == 1) + 1:nnz(yOPTIM == 1) + nnz(zOPTIM == 1) + nnz(pOPTIM == 1));
        s_(sOPTIM == 1) = x_(nnz(yOPTIM == 1) + nnz(zOPTIM == 1) + nnz(pOPTIM == 1) + 1:nnz(yOPTIM == 1) + nnz(zOPTIM == 1) + nnz(pOPTIM == 1) + nnz(sOPTIM == 1));
    end


    function x_ = MapYZPStoX(y_, z_, p_, s_)
        % Compose the optimization vector from the individual components
        % that are being optimized:
        % The optimization vector x consists of the following variables:
        % - yINIT(yOPTIM==1)
        % - zINIT(zOPTIM==1)
        % - pINIT(pOPTIM==1)
        % - sINIT(sOPTIM==1)
        x_ = [y_(yOPTIM == 1);
              z_(zOPTIM == 1);
              p_(pOPTIM == 1);
              s_(sOPTIM == 1)];
    end


    function [yOUT_, zOUT_] = RunSimulation(x_)
        % This function runs all partial integrations necessary to create a
        % full stride. It returns the final values of the continuous and
        % discrete states.
        [yIN_, zIN_, p_, s_] = MapXtoYZPS(x_); % The optimization vector x_ is mapped into states and parameters
        % Check if simulation is actually necessary, or if we can just re-use the old values:
        if ~isempty(yIN_OLD) && all(yIN_ == yIN_OLD) && all(zIN_ == zIN_OLD) && all(p_ == p_OLD) && all(s_ == s_OLD)
            % Just use the old values.
            yOUT_ = yOUT_OLD;
            zOUT_ = zOUT_OLD;
        else
            options_.tIN  = 0;
            options_.tMAX = tMAX;
            if activeSyst
                [yOUT_, zOUT_, tOUT_] = hybrDynHndl(yIN_, zIN_, p_, exctFcnHndl, s_, options_);
            else
                [yOUT_, zOUT_, tOUT_] = hybrDynHndl(yIN_, zIN_, p_, options_);
            end
            % Check if the simulation finished:
            if (tOUT_<0)
%                 disp('The integration ran out of time. If this message appears multiple')
%                 disp('times and the search does not converge, try increasing options.tMAX!');
            end 
        end
        % remember the old values for sections that do not need further
        % integration:
        yIN_OLD = yIN_;
        zIN_OLD = zIN_;
        p_OLD = p_;
        s_OLD = s_;
        yOUT_OLD = yOUT_;
        zOUT_OLD = zOUT_;
    end


    function ceq_ = RootTransferFunction(x_)
        % Compute the difference between initial and terminal states for
        % all continuous and discrete states that need to be periodic:
        try % Run simulation
            [yOUT_, zOUT_] = RunSimulation(x_);
        catch le % The simulation returned an error. 
            % Set the cost function to an arbitrarily high value:
            disp(['The integration could not be finished due to the following error: ',le.message]);
            disp(' -> Setting the root search function to an arbitrary high value');
            ceq_ = 1e12*ones(nnz(yPERIOD == 1) + nnz(zPERIOD == 1),1);
            return
        end
        % Periodicity as required by the user:
        [yIN_, zIN_] = MapXtoYZPS(x_);
%         ceq_ = [yOUT_(yPERIOD == 1) - yIN_(yPERIOD == 1);... 
%                 zOUT_(zPERIOD == 1) - zIN_(zPERIOD == 1)];
        if zINIT(1) ==2
          % Double stance
          DisT = norm(yCYC_OLD([2,4,7,9]) - yOUT_([2,4,7,9])); % distance in poincare section
        else
          % Single stance  
          DisT = norm(yCYC_OLD([2,4,7,8,9]) - yOUT_([2,4,7,8,9]));
%           DisT = norm(yCYC_OLD([2,4]) - yOUT_([2,4])); % distance in poincare section  
        end  
%         DisT = norm(yINIT([2,4,7,9]) - yOUT_([2,4,7,9])); 
        ceq_ = [yOUT_(yPERIOD == 1) - yIN_(yPERIOD == 1);... 
                zOUT_(zPERIOD == 1) - zIN_(zPERIOD == 1);...
                DisT - ALPstep]; 
        % Move 0.1 along the arc of the solution;   
    end


    function f_ = CostFunction(x_)
        [~, ~, p_, s_] = MapXtoYZPS(x_);
        % Compute the cost of the given guess 'x_'
        try % Run simulation
            [yOUT_, zOUT_] = RunSimulation(x_);
        catch le % The simulation returned an error. 
            % Set the cost function to an arbitrarily high value:
            disp(['The integration could not be finished due to the following error: ',le.message]);
            disp('Setting the cost function to an arbitrary high value');
            f_ = 1e12;
            return
        end
        f_ = sum(yOUT_.*yCOST) + sum(zOUT_.*zCOST) + sum(p_.*pCOST) + sum(s_.*sCOST);
    end


    % This function maps the flags for periodicity into a multidimensional
    % nonlinear equality constraint. 
    function [c_, ceq_] = ConstrainFunction(x_)
        % No inequality constraints exist:
        c_ = []; 
        % Call root-function for equality constraints:
        ceq_ = RootTransferFunction(x_);
    end
    % *********************************************************************
end
% *************************************************************************
% *************************************************************************