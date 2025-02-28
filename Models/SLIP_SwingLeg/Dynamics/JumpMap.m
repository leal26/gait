% *************************************************************************
%
% function [yPLUS, zPLUS, isTerminal] = JumpMap(yMINUS, zMINUS, p, event)
% 
% This MATLAB function defines the discrete dynamics of a hybrid dynamic
% model of a simple SLIP (Spring Loaded Inverted Pendulum) model in 2D. The
% model's current continuous and discrete states before an event (together
% with the model parameters) are provided by the calling routine to which
% the states after the event are returned.
%
% Input:  - A vector of continuous states before the event 'yMINUS' 
%         - A vector of discrete states before the event 'zMINUS' 
%         - A vector of model system parameters 'p'
%
% Output: - A vector of continuous states after the event 'yPLUS' 
%         - A vector of discrete states after the event 'zPLUS' 
%         - A Boolean flag that indicates if the current event will 
%           terminate the simulation or not.
%
% Created by C. David Remy on 07/10/2011
% MATLAB 2010a - Windows - 64 bit
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
%   See also HYBRIDDYNAMICS, FLOWMAP, JUMPSET, COMPUTEDIFFERENTIABLEFORCES 
%            CONTSTATEDEFINITION, DISCSTATEDEFINITION, SYSTPARAMDEFINITION,
%            EXCTSTATEDEFINITION, EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC, 
%            SYMBOLICCOMPUTATIONOFEQM. 
%

function [yPLUS, zPLUS, isTerminal] = JumpMap(yMINUS, zMINUS, p, event)
    
    % Get a mapping for the state and parameter vectors.  This allows us
    % to use a more readable syntax: "y(contStateIndices.dy)" instead of
    % "y(3)" while still operating with vectors and not with structs.
    % We keep the index-structs in memory to speed up processing
    persistent contStateIndices  systParamIndices discStateIndices
    global active_leg
    global period
    global right_TD
    global prev_Y_R
    global prev_right_TD
    global current_Y_R
    global prev_Y_L
    global current_Y_L
    if isempty(contStateIndices)  || isempty(systParamIndices) || isempty(discStateIndices)
        [~, ~, contStateIndices] = ContStateDefinition();
        [~, ~, systParamIndices] = SystParamDefinition();
        [~, ~, discStateIndices] = DiscStateDefinition();
    end
    
    % As most things remain unchanged we copy the incoming states
    yPLUS = yMINUS;
    zPLUS = zMINUS;
    phiL=yMINUS(contStateIndices.phiL);
    phiR=yMINUS(contStateIndices.phiR);
    alpha=yMINUS(contStateIndices.alpha);

    % ************************************
    % ************************************
    % Only discrete states are altered!
    %
    % Event 1: Detect touchdown left
    % Event 2: Detect liftoff left
    % Event 4: Detect touchdown right
    % Event 5: Detect liftoff right
    
    % Left leg is in blue
    % Right leg is in grey
    l_legL = p(systParamIndices.l_0);
    r_legL = p(systParamIndices.l_0);
    
    switch event
        case 1 % Event 1: Detect touchdown left
            
            %%%%%%%%%%%%%%% For symmertical gaits
            % Record left leg states
            % z_lcontPt   = yMINUS(contStateIndices.x) + l_legL * sin(phiL + alpha);
            % zPLUS_phiL  = 2;
            % yPLUS_phiL  = atan2(z_lcontPt-yMINUS(contStateIndices.x), yMINUS(contStateIndices.y)-0) - alpha ;
            % yPLUS_dphiL = -yMINUS(contStateIndices.dx)*cos(yPLUS_phiL + alpha) - yMINUS(contStateIndices.dy)*sin(yPLUS_phiL + alpha);
            
            % Only simulate half of the stride. Switch left and right legs 
            % states to make them periodic. (The order of the following code 
            % matters!)
%             yPLUS(contStateIndices.phiL)  =  yMINUS(contStateIndices.phiR);
%             yPLUS(contStateIndices.dphiL) =  yMINUS(contStateIndices.dphiR);
%             yPLUS(contStateIndices.phiR)  =  yPLUS_phiL;
%             yPLUS(contStateIndices.dphiR) =  yPLUS_dphiL; % comment these lines for asymmerical skipping solutions  
%             
%             zPLUS(discStateIndices.lcontPt) = zPLUS(discStateIndices.rcontPt) - yMINUS(contStateIndices.x);
%             zPLUS(discStateIndices.rcontPt) = z_lcontPt - yMINUS(contStateIndices.x);
%             zPLUS(discStateIndices.lphase)  = zPLUS(discStateIndices.rphase);
%             zPLUS(discStateIndices.rphase)  = zPLUS_phiL;
%             % Reset x to zero, so x is periodic as well.
%             yPLUS(contStateIndices.x) = 0; 
            zPLUS(discStateIndices.lcontPt) = yMINUS(contStateIndices.x) + l_legL * sin(phiL + alpha);
            zPLUS(discStateIndices.lphase)  = 2;
            yPLUS(contStateIndices.phiL)    = atan2(zPLUS(discStateIndices.lcontPt)-yPLUS(contStateIndices.x), yPLUS(contStateIndices.y)-0) - alpha;
            yPLUS(contStateIndices.dphiL)   = -yMINUS(contStateIndices.dx)*cos(yPLUS(contStateIndices.phiL) + alpha) - yMINUS(contStateIndices.dy)*sin(yPLUS(contStateIndices.phiL) + alpha);
            % Intermediate event. Simulation continues    
            %%%%%%%%%%%%%% For symmertical gaits
            isTerminal = false;  
            prev_Y_L = current_Y_L;
            current_Y_L = yMINUS(2:end-1);     

        case 2 % Event 2: Detect liftoff left
            zPLUS(discStateIndices.lphase)  = 1;
            % Reset left leg angular velocity at liftoff to stance velocity
            yPLUS(contStateIndices.phiL)    = atan2(zPLUS(discStateIndices.lcontPt)-yPLUS(contStateIndices.x), yPLUS(contStateIndices.y)-0) - alpha;
            yPLUS(contStateIndices.dphiL)   = -yMINUS(contStateIndices.dx)*cos(yPLUS(contStateIndices.phiL) + alpha) - yMINUS(contStateIndices.dy)*sin(yPLUS(contStateIndices.phiL) + alpha);
            % Intermediate event. Simulation continues
            isTerminal = false; 
            
            
        case 3 % Event 3: detect Apex 1 (apex transit: dy==0 during flight)
            % Intermediate event. Simulation continues
            isTerminal = false; 
            if strcmp(active_leg, 'left')
                active_leg = 'right';
            elseif strcmp(active_leg, 'right')
                active_leg = 'left';
            end
            

        case 4 % Event 4: Detect touchdown right
            zPLUS(discStateIndices.rcontPt) = yMINUS(contStateIndices.x) + r_legL * sin(phiR + alpha);
            zPLUS(discStateIndices.rphase)  = 2;
            yPLUS(contStateIndices.phiR)    = atan2(zPLUS(discStateIndices.rcontPt)-yPLUS(contStateIndices.x), yPLUS(contStateIndices.y)-0) - alpha;
            yPLUS(contStateIndices.dphiR)   = -yMINUS(contStateIndices.dx)*cos(yPLUS(contStateIndices.phiR) + alpha) - yMINUS(contStateIndices.dy)*sin(yPLUS(contStateIndices.phiR) + alpha);
            % Intermediate event. Simulation continues
            isTerminal = false;  
            period = yMINUS(contStateIndices.t) - right_TD;
%             disp('here')
%             disp(period)
            prev_right_TD = right_TD;
            right_TD = yMINUS(contStateIndices.t);
            prev_Y_R = current_Y_R;
            current_Y_R = yMINUS(2:end-1);
%              disp('right_TD')
%              disp(right_TD)
%             disp('current_Y_R')
%             disp(current_Y_R)
            
        case 5 % Event 5: Detect liftoff right
            zPLUS(discStateIndices.rphase)  = 1;
            % Reset right leg angular velocity at liftoff to stance velocity
            yPLUS(contStateIndices.phiR)    = atan2(zPLUS(discStateIndices.rcontPt)-yPLUS(contStateIndices.x), yPLUS(contStateIndices.y)-0) - alpha; 
            yPLUS(contStateIndices.dphiR)   = -yMINUS(contStateIndices.dx)*cos(yPLUS(contStateIndices.phiR) + alpha) - yMINUS(contStateIndices.dy)*sin(yPLUS(contStateIndices.phiR) + alpha);
            % Intermediate event. Simulation continues
            isTerminal = false;
            
            
        case 6 % Event 6: detect Apex 2 (apex transit: dy==0 during flight)
            % Intermediate event. Simulation continues
            isTerminal = false;  
%             if strcmp(active_leg, 'left')
%                 active_leg = 'right';
%             elseif strcmp(active_leg, 'right')
%                 active_leg = 'left';
%             end
    end
    
    % ************************************
    % ************************************
end
% *************************************************************************
% *************************************************************************