% *************************************************************************
%
% function evntVal = JumpSet(y, z, p)
%
% This MATLAB function defines the occurrence of discrete events that
% change the dynamics of a simple SLIP (Spring Loaded Inverted Pendulum)
% model in 2D. The model's current continuous and discrete states together
% with the model parameters are provided by the calling routine to which a
% vector of event function values is returned. The directional
% zero-crossings of these functions trigger each a different event. 
%
% Input:  - A vector of continuous states 'y' 
%         - A vector of discrete states 'z' 
%         - A vector of model system parameters 'p'
%
% Output: - Each entry of 'evntVal' corresponds to a function, of which a
%           zero-crossing (with positive derivative) is detected as event
%
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
%   See also HYBRIDDYNAMICS, FLOWMAP, JUMPMAP, COMPUTEDIFFERENTIABLEFORCES 
%            CONTSTATEDEFINITION, DISCSTATEDEFINITION, SYSTPARAMDEFINITION,
%            EXCTSTATEDEFINITION, EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC, 
%            SYMBOLICCOMPUTATIONOFEQM. 
%
function evntVal = JumpSet(y, z, p, varargin)
   
	% Get a mapping for the state and parameter vectors.  This allows us
    % to use a more readable syntax: "y(contStateIndices.dy)" instead of
    % "y(3)" while still operating with vectors and not with structs.
    % We keep the index-structs in memory to speed up processing
    persistent contStateIndices  systParamIndices discStateIndices
    global active_leg
    if isempty(contStateIndices)  || isempty(systParamIndices) || isempty(discStateIndices)
        [~, ~, contStateIndices] = ContStateDefinition();
        [~, ~, systParamIndices] = SystParamDefinition();
        [~, ~, discStateIndices] = DiscStateDefinition();
    end
    
    phiL  = y(contStateIndices.phiL);
    phiR  = y(contStateIndices.phiR);
    alpha = y(contStateIndices.alpha);
    % NOTE:  your event function must go from negative to positive to
    % trigger an event!
    %
    % Event 1: Detect touchdown
    % Event 2: Detect liftoff
    % Event 3: Detect stop (apex transit: dy==0 during flight)
   
    n_events = 5;
    evntVal = zeros(n_events,1);
    
    l_legL = p(systParamIndices.l_0);
    r_legL = p(systParamIndices.l_0);
    
    v_footl = y(contStateIndices.dy) + y(contStateIndices.dphiL)*l_legL*sin(y(contStateIndices.phiL));
    v_footr = y(contStateIndices.dy) + y(contStateIndices.dphiR)*r_legL*sin(y(contStateIndices.phiR));
    
    % Event 1: Detect touchdown left 
    % Change the guard function to select different events
    if (z(discStateIndices.lphase) == 1) && (z(discStateIndices.rphase) == 1) && (v_footl < 0) && strcmp(active_leg,'left')
        ft_height = y(contStateIndices.y) - l_legL * cos(alpha + phiL);
        evntVal(1) = -ft_height;
    else
        % Only detect this event during flight
        evntVal(1) = -1;
    end
    
    % *******
    % Event 2: Detect liftoff left
    if z(discStateIndices.lphase) == 2 && strcmp(active_leg,'left')%(i.e., in stance) 
        l_leg = sqrt((y(contStateIndices.x)-z(discStateIndices.lcontPt))^2 + (y(contStateIndices.y)-0)^2);
        evntVal(2) =l_leg - l_legL;
    else
        % Only detect this event during stance
        evntVal(2) = -1;
    end
    
    % *******
    % Event 3: Detect Apex transit 1
    if   (z(discStateIndices.lphase) == 1) && (z(discStateIndices.rphase) == 1)
         evntVal(3) = -y(contStateIndices.dy);
    else
        % Only detect this event during stance
        evntVal(3) = -1; 
    end

    % *******
    % Event 4: Detect touchdown right
    % Change the guard function to enable th
    if (z(discStateIndices.rphase) == 1)   && (v_footr < 0)  && strcmp(active_leg,'right')
        ft_height = y(contStateIndices.y) - r_legL * cos(alpha + phiR);
        evntVal(4) = -ft_height;

    else
        % Only detect this event during flight
        evntVal(4) = -1;
    end
    
    % *******
    % Event 5: Detect liftoff right
    if (z(discStateIndices.rphase) == 2) && strcmp(active_leg,'right')
        l_leg = sqrt((y(contStateIndices.x)-z(discStateIndices.rcontPt))^2 + (y(contStateIndices.y)-0)^2);
        evntVal(5) =l_leg - r_legL;
    else
        % Only detect this event during stance
        evntVal(5) = -1;
    end

    % Event 6: Detect left leg zero angle
    % This event is triggered in stance phase
    if (z(discStateIndices.lphase) == 2) 
        l_leg_Angle = y(contStateIndices.phiL);
        evntVal(6) = l_leg_Angle;
    else
        % Only detect this event during stance
        evntVal(6) = -1;
    end


    %
    % ************************************
    % ************************************
end
% *************************************************************************
% *************************************************************************