% *************************************************************************
%
% function z = DiscStateDefinition()
%
% This MATLAB function defines the discrete state vector 'z' for a simple
% SLIP (Spring Loaded Inverted Pendulum) model in 2D.  Besides serving as
% initial configuration of the model, this file provides a definition of
% the individual components of the discrete state vector and an index
% struct that allows name-based access to its values.
%
% NOTE: This function is relatively slow and should not be executed within
%       the simulation loop.
%
% Input:  - NONE
% Output: - The initial discrete states as the vector 'discStateVec' (or 'z')
%         - The corresponding state names in the cell array 'discStateNames' 
%         - The struct 'discStateIndices' that maps these names into indices  
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
%            CONTSTATEDEFINITION, SYSTPARAMDEFINITION, EXCTSTATEDEFINITION,
%            EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC. 
%
function [discStateVec, discStateNames, discStateIndices] = DiscStateDefinition()
    
    discState.lphase  = 1; % ['1','2'] The current phase of the left leg  (flight = 1) (stance = 2) 
    discState.lcontPt = 0; % [l_0]     Horizontal position of the last ground contact for left leg
    
    discState.rphase  = 1; % ['1','2'] The current phase of the model  (flight = 1) (stance = 2) 
    discState.rcontPt = 0; % [l_0]     Horizontal position of the last ground contact for right leg
    
    discState.E       = 0; 
    discState.TDz1    = 0; 
    discState.TDz2    = 0; 

    
    [discStateVec, discStateNames] = Struct2Vec(discState);
    discStateIndices = Vec2Struct(1:1:length(discStateVec),discStateNames);
end