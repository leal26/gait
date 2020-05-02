% *************************************************************************
%
% function [f, v, c] = GetUpperSpringPatch()
% 
% Returns faces, vertices, and color-values (one per face) that represent a
% patch of the upper spring of a prismatic leg.
%
% - It is pointing towards [0, 0, 1] (on a circle around the origin)
% - The uncompressed spring length is 0.25
% - The spring is positioned at [-0.2427, 0, 0] (cos(atan2(0.1,0.26))*0.26)
% - It is pointing towards [0, 0, 1]
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
function [f, v, c] = GetUpperSpringPatch()
    % Create spring    
    [x, y, z] = GetSpringSurface(20, 100, 0.25, 0.085, 0.01 ,7);
    [f, v] = surf2patch(x, y, z, z);
    % Color the spring in red
    c = repmat([1 0 0], length(f), 1);
    % Actual spring length is 0.25
    % The spring is shifted laterally, such that it is prepared for
    % bending (which is done in the calling routine):
    v = TransformVertices(v, diag([1,1,1]), [-cos(atan2(0.1,0.26))*0.26, 0, 0]);
end
% *************************************************************************
% *************************************************************************