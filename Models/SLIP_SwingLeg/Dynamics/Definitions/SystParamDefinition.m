% *************************************************************************
%
% function [systParamVec, systParamNames, systParamIndices] = SystParamDefinition()
% function p = SystParamDefinition()
%
% This MATLAB function defines the physical system parameter vector 'p' for
% a simple SLIP (Spring Loaded Inverted Pendulum) model in 2D.  Besides
% serving as initial configuration of the model, this file provides a
% definition of the individual components of the system parameter vector
% and an index struct that allows name-based access to its values.
%
% NOTE: This function is relatively slow and should not be executed within
%       the simulation loop.
%
% Input:  - NONE
% Output: - The initial system parameters as the vector 'systParamVec' (or 'p')
%         - The corresponding parameter names in the cell array 'systParamNames' 
%         - The struct 'systParamIndices' that maps these names into indices  
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
%     buffintk@bucknell.edu
%
% (2) Department of Mechanical Engineering, 
%     Bucknell University
%     701 Moore Avenue
%     Lewisburg, PA-17837, USA
%
%   See also HYBRIDDYNAMICS, FLOWMAP, JUMPMAP, JUMPSET, 
%            CONTSTATEDEFINITION, DISCSTATEDEFINITION, EXCTSTATEDEFINITION,
%            EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC. 
%
function [systParamVec, systParamNames, systParamIndices] = SystParamDefinition()
    % ************************************
    % ************************************
    % TASK 5: EXECUTE THIS FUNCTION 
    % ************************************
    % Then access the leg stiffness with
    % systParamVec(systParamIndices.k)
    % ************************************
    % ************************************

    % All units are normalized to gravity g, total mass m_0, and
    % uncompressed leg length l_0.
    
    % Physics:
    systParam.g         = 1;     % [*] gravity
    % Parameter of the model
    systParam.l_0       = 1;     % [*] uncompressed leg length
    systParam.m_0       = 1;     % [m_0] total mass
    systParam.k         = 20;    % [m_0*g/l_0] linear spring stiffness in the leg
    systParam.kh        = 5;     % [m_0*g/l_0] linear spring stiffness in the leg
    systParam.st        = 0;     % 
    systParam.Llo       = 0;     % 
   

    [systParamVec, systParamNames] = Struct2Vec(systParam);
    systParamIndices = Vec2Struct(1:1:length(systParamVec),systParamNames);
    systParamIndices.SMA_L = struct('T',{});
    systParamIndices.SMA_R = struct('T',{});
end