% *************************************************************************
%
% function [contStateVec, contStateNames, contStateIndices] = ContStateDefinition()
% function y = ContStateDefinition()
%
% This MATLAB function defines the continuous state vector 'y' for a simple
% SLIP (Spring Loaded Inverted Pendulum) model in 2D.  Besides serving as
% initial configuration of the model, this file provides a definition of
% the individual components of the continuous state vector and an index
% struct that allows name-based access to its values.
%
% NOTE: This function is relatively slow and should not be executed within
%       the simulation loop.
%
% Input:  - NONE
% Output: - The initial continuous states as the vector 'contStateVec' (or 'y')
%         - The corresponding state names in the cell array 'contStateNames' 
%         - The struct 'contStateIndices' that maps these names into indices  
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
%   See also HYBRIDDYNAMICS, FLOWMAP, JUMPMAP, JUMPSET, 
%            DISCSTATEDEFINITION, SYSTPARAMDEFINITION, EXCTSTATEDEFINITION,
%            EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC. 
%
function [contStateVec, contStateNames, contStateIndices] = ContStateDefinition()
    % All units are normalized to gravity g, total mass m_0, and
    % uncompressed leg length l_0.
    
    contState.x       = 0;   % [l_0] horizontal position of the main body CoG
    contState.dx      = 0;   % [sqrt(g*l_0)] ... velocity thereof
    contState.y       = 1;   % [l_0] vertical position of the main body CoG
    contState.dy      = 0;   % [sqrt(g*l_0)] ... velocity thereof
    contState.alpha   = 0;   % [rad] main body orientation
    contState.dalpha  = 0;   % [rad/s] ... velocity thereof
    contState.phiL    = 0;   % [rad] left leg angle
    contState.dphiL   = 0;   % [rad/s] ... velocity thereof 
    contState.phiR    = 0;   % [rad] right leg angle
    contState.dphiR   = 0;   % [rad/s] ... velocity thereof  
    contState.t       = 0;   % [sqrt(l_o/g)] Time    
    
    
    [contStateVec, contStateNames] = Struct2Vec(contState);
    contStateIndices = Vec2Struct(1:1:length(contStateVec),contStateNames);
end