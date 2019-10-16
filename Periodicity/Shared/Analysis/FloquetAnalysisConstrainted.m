% % *************************************************************************
%
% function [eigenValuesCYC, eigenVectorsCYC, Jx, Js] = ...
%            FloquetAnalysis(hybrDynHndl, yCYC,    zCYC,   pCYC, exctFcnHndl, sCYC, 
%                                         yANALYS, zANALYS,                   sANALYS)
%
% function [eigenValuesCYC, eigenVectorsCYC, Jx] = ...
%            FloquetAnalysis(hybrDynHndl, yCYC,    zCYC,   pCYC, 
%                                         yANALYS, zANALYS)
%
% This function returns the Eigenvalues and Eigenvectors of the Floquet
% analysis of a periodic solution for the model described by 
% 'hybrDynHndl', as well as the complete Jacobian, i.e. numeric
% differential for selected states (yANALYS, zANALYS) and excitation
% parameters (sANALYS). The model can be passive or active.
%
% For a combined solution vector x containing selected elements of y
% and z (where the flags yANALYS and zANALYS are set) and a combined
% transfer function x(k+1) = P(x(k)) with periodic solution xCYC = P(xCYC)
% we can approximate:  
%
% P(x) = xCYC + Jx*(xCYC-x) + O2(x)
%
% If we consider additionally a driven system, this can be expanded with a
% vector s of selected excitation parameters (where the flag sANALYS is
% set):
%
% P(x,s) = xCYC + Jx*(xCYC-x) + Js*(sCYC-s) + O2(x,s)
%
% Input:  - A one stride gait model with one of the following interfaces:
%           [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, exctFcnHndl, s, options)
%           [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, options)
%           This function maps a set of initial states to a set of final
%           states, using the given parameters. The user must support the
%           function handle 'hybrDynHndl' to this model.
%         - A periodic solution for the continuous states 'yCYC' (Column
%           vector)
%         - A periodic solution for the discrete states 'zCYC' (Column
%           vector)
%         - A periodic solution for the parameter 'pCYC' (Column vector)
%         - OPTIONAL: A ExcitationFunctionHandle that is used by the
%           OneStrideDynamics function.
%         - OPTIONAL: A set of parameters 'sCYC' for this function (These
%           parameters should result in a periodic solution)
%         - A vector of flags 'yANALYS', indicating which continuous
%           states should be disturbed [flag = 1] (Column vector)
%         - A vector of flags 'zANALYS', indicating which discrete 
%           states should be disturbed [flag = 1] (Column vector)
%         - OPTIONAL: A vector of flags 'sANALYS', indicating which excitation  
%           parameters should be disturbed [flag = 1] (Column vector)
%
% Finally, additional options can be provided by an 'options' struct. The
% following options are provided: 
% - options.tMAX [10] Maximal simulation time, after which the integration
%                     is aborted.  If this happens, an error is issued. 
% - options.disturbance [1e-6] Numerical disturbance used in the
%                              computation of the Monodromy matrix. 
%
% function [eigenValuesCYC, eigenVectorsCYC, Jx, Js] = ...
%            FloquetAnalysis(hybrDynHndl, yCYC,    zCYC,   pCYC, exctFcnHndl, sCYC, 
%                                         yANALYS, zANALYS,                   sANALYS, 
%                                         options)
% function [eigenValuesCYC, eigenVectorsCYC, Jx] = ...
%            FloquetAnalysis(hybrDynHndl, yCYC,    zCYC,   pCYC, 
%                                         yANALYS, zANALYS, 
%                                         options)
%
% Output: - Eigenvalues 'eigenValuesCYC' and corresponding Eigenvectors
%           'eigenVectorsCYC' of the monodromy matrix, and the partial
%           deriviative J.
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
%   See also HYBRIDDYNAMICS, FINDPERIODICSOLUTION.
%    
function [eigenValuesCYC, eigenVectorsCYC, varargout] = FloquetAnalysisConstrainted(hybrDynHndl, yCYC, zCYC, pCYC, varargin)
    
    % *********************************************************************
    % Input handling
    if ~(isa(varargin{end},'struct'))
        if (nargin~=6) && (nargin~=9)
            error('GaitCreation:FloquetAnalysis:wrongParameterCount', 'Wrong number of input arguments.  Should be 6 for a passive system and 9 for an active system')
        end
    else
        if (nargin~=7) && (nargin~=10)
            error('GaitCreation:FloquetAnalysis:wrongParameterCount', 'Wrong number of input arguments.  Should be 7 for a passive system and 10 for an active system, when an options struct is provided')
        end
    end
    if (nargin==6 || nargin==7) % Passive system
        sCYC = [];
        yANALYS = varargin{1};
        zANALYS = varargin{2};
        sANALYS = [];
        passive = 1;
    else % Active system
        exctFcnHndl = varargin{1};
        sCYC = varargin{2};
        yANALYS = varargin{3};
        zANALYS = varargin{4};
        sANALYS = varargin{5};
        passive = 0;
    end
     % Check if options were provided:
    options = struct([]);
    if (isa(varargin{end},'struct'))
        options = varargin{end};
    end
    % Evaluate options:
    if isfield(options,'tMAX')
        tMAX = options.tMAX;
    else
        tMAX = 15;
    end
    if isfield(options,'disturbance')
        disturbance = options.disturbance;
    else
        % This disturbance is applied to all selected states, to compute linear
        % finite central differences:     
        disturbance = 1e-6; 
    end
    % Create index vectors to all states and parameters that should be disturbed
    yIndex = find(yANALYS == 1);
    zIndex = find(zANALYS == 1);
    sIndex = find(sANALYS == 1);
    % Create some detailed information for the screen:
    disp('Start limit cycle analysis...');
    if passive 
        disp(' ... for a passive system');
    else
        disp(' ... for an active system');
    end
    if ~isempty(yCYC(yANALYS == 1))
        disp('  Disturbing the following continuous DOFs:');
        [~,contStateNames] = ContStateDefinition();
        disp(contStateNames(yANALYS == 1));
    end
    if ~isempty(zCYC(zANALYS == 1))
        disp('  Disturbing the following discrete DOFs:');
        [~,discStateNames] = DiscStateDefinition();
        disp(discStateNames(zANALYS == 1));
    end
    if ~isempty(sCYC(sANALYS == 1))
        disp('  Disturbing the following controller parameters:');
        [~,exctParamNames] = ExctParamDefinition();
        disp(exctParamNames(sANALYS == 1));
    end
    disp('  Computing:');
    disp('  - Eigenvalues');
    if (nargout > 1)
        disp('  - Eigenvectors');
    end
    if (nargout > 2)
        disp('  - Jacobian');
    end
    % *********************************************************************
    
    
    % *********************************************************************
    % Compute first order derivative
    % Compute the total number of dimensions that will be disturbed:
    nrDists = length(yIndex) + length(zIndex) + length(sIndex);
    % Create a combined vector of initial states 
    yzsIN = [yCYC(yIndex);zCYC(zIndex);sCYC(sIndex)];
    % .. and disturbe them in the positive direction:
    yzsPLUSIN = repmat(yzsIN, 1, nrDists) + disturbance*eye(nrDists);
    % ...and negative direction:
    yzsMINUSIN = repmat(yzsIN, 1, nrDists) - disturbance*eye(nrDists);
    % Simluate all these cases
    yzPLUSOUT  = combinedSystem(yzsPLUSIN);
    yzMINUSOUT = combinedSystem(yzsMINUSIN);
    % Compute an overall Jacobi-Matrix using central differences
    Jyzs = (yzPLUSOUT - yzMINUSOUT) / (2*disturbance);
    % Extract the two sub-matrices for states and parameters:
    if ~isempty([yCYC(yANALYS == 1);zCYC(zANALYS == 1)])
        Jx = Jyzs(:,1:length(yIndex) + length(zIndex));
    end
    if ~isempty(sCYC(sANALYS == 1))
        Js = Jyzs(:,length(yIndex) + length(zIndex)+1:end);
    end
    % Compute the eigenvalues and eigenvectors for Jx
    [eigenVectorsCYC, D] = eig(Jx);

   

    eigenValuesCYC  = diag(D);
    disp('...found the following Eigenvalues:');
    for i = 1:length(eigenValuesCYC)
        disp(['  Eigenvalue ',num2str(i),' = ',num2str(eigenValuesCYC(i))]);
    end
    % Provide required output
    if (nargout>2)
        varargout(1) = {Jx};
        if (nargin == 9  || nargin == 11) 
            if ~isempty(sCYC(sANALYS == 1)) % Active System
                varargout(2) = {Js};
            end
         else
                varargout(2) = {[]}; % Passive System   % ZG: Modified on 11/9      
        end
    end
    % Done
    disp('...done');
    % *********************************************************************
    
    
    
    % *********************************************************************
    % Local functions:
    % This function evaluates the Hybrid Dynamic Handle for a combined
    % vector of y, z, and s entries that have been selected from the full
    % state descriptions according to yANALYS, zANALYS, sANALYS.  Missing
    % elements are padded with the values from the cyclic solutions.
    % Multiple initial states can be processed at once, if they are given
    % as matrix (one column per state)
    function yzOUT_ = combinedSystem(yzsIN_)
        n_ = size(yzsIN_,2);
        yzOUT_ = zeros(length(yIndex) + length(zIndex), n_);
        for i_=1:n_
            % copy the originals (as we have to pad the copies with their
            % values anyway) 
            yIN_ = yCYC;
            zIN_ = zCYC;
            sIN_ = sCYC;
            % Modify the states that are defined by yANALYS, zANALYS, and
            % sANALYS (and hence contained in yzsIN_)
            if ~isempty(yCYC(yANALYS == 1))
                yIN_(yIndex) = yzsIN_(1:length(yIndex), i_);
            end
            if ~isempty(zCYC(zANALYS == 1))
                zIN_(zIndex) = yzsIN_(length(yIndex)+1:length(yIndex)+length(zIndex), i_);
            end
            if ~isempty(sCYC(sANALYS == 1))
                sIN_(sIndex) = yzsIN_(length(yIndex)+length(zIndex)+1:end, i_);
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % modified to conduct analysis for double stance phase by
            [~, ~, contStateIndices] = ContStateDefinition();
            [~, ~, discStateIndices] = DiscStateDefinition();
            % reset the other states at here
            % Causality issue here: should we reset y or leg length

            % Always reset right leg values, leg lenght is 1 at touchdown;
            zIN_(discStateIndices.rcontPt) = 1*sin(yIN_(contStateIndices.phiR)); 
            yIN_(contStateIndices.dphiR)   = - yIN_(contStateIndices.dx)*cos(yIN_(contStateIndices.phiR))/1 ...
                                             - yIN_(contStateIndices.dy)*sin(yIN_(contStateIndices.phiR))/1; 
            
            % update vertical position y            
            yIN_(contStateIndices.y) = cos(yIN_(contStateIndices.phiR))*1; 
            
            if zIN_(discStateIndices.lphase) == 2 % if left leg is in stance
                % rest left leg contact position, leg length, and leg
                % rotational velocity;
                zIN_(discStateIndices.lcontPt) =   yIN_(contStateIndices.y)*tan(yIN_(contStateIndices.phiL));
                LegL                           =   yIN_(contStateIndices.y)/cos(yIN_(contStateIndices.phiL));
                yIN_(contStateIndices.dphiL)   = - yIN_(contStateIndices.dx)*cos(yIN_(contStateIndices.phiL))/LegL ...
                                                 - yIN_(contStateIndices.dy)*sin(yIN_(contStateIndices.phiL))/LegL;
            end
             
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % simulate
            options_.tMAX = tMAX;
            if passive 
                [yOUT_, zOUT_, tOUT_] = hybrDynHndl(yIN_, zIN_, pCYC, options);
            else
                [yOUT_, zOUT_, tOUT_] = hybrDynHndl(yIN_, zIN_, pCYC, exctFcnHndl, sIN_, options_);
            end
            if (tOUT_<0)
                disp('The integration ran out of time. If this error appears multiple')
                disp('times supply options.tMAX.  If it only appears occasinally, it ')
                disp('might indicate that the problem is ill-conditioned');
            end 
            % Map outputs
            yzOUT_(:,i_) = [yOUT_(yIndex,:);zOUT_(zIndex,:)];
        end
    end
    % *********************************************************************
end
% *************************************************************************
% *************************************************************************