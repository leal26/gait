% *************************************************************************
%
% function [f, v, c] = GetUpperActuatorPatch()
% 
% Returns faces, vertices, and color-values (one per face) that represent a
% patch of the upper actuator of a prismatic leg.
%
% - The actuator is positioned at [-0.2427, 0, 0] (cos(atan2(0.1,0.26))*0.26)
% - It is pointing towards [0, 0, 1] (on a circle around the origin)
%
% Input:  - none
%
% Output: - faces 'f', vertices 'v', and color values 'c' as used in the
%           'patch' function
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
%   See also GETMONOPED2DOFPATCH, PATCH.
%
function [f, v, c] = GetUpperActuatorPatch()
    % The actuator is created from a cylinder:
    [x, y] = cylinder([0.043, 0.043, 0.043, 0.053, 0.1-0.047*(cos((0:1:10)/10*pi/2)), 0.1 , 0.1, 0.1, 0.043, 0.043, 0.043]);
    z = repmat([0.05; 0.057;0.057;0.057;0.057-0.047*(sin((0:1:10)/10*pi/2))';0.01;0;0;0;0;0.05], 1, length(x));
    [f, v] = surf2patch(x, y, z, z);
    % Set a uniform grey color
    c = ones(length(f), 3)*0.95;
    % color two segments black
    c(16:20:400, :) = repmat([0 0 0], 20, 1);
    c(18:20:400, :) = repmat([0 0 0], 20, 1);
    % transform laterally and bend!
    v = TransformVertices(v, diag([1,1,-1]), [-cos(atan2(0.1,0.26))*0.26, 0, 0]);
    % Bend:
    v = BendVertices(v,0.25/( 1.2036)*2*pi);
end
% *************************************************************************
% *************************************************************************