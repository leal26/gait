% *************************************************************************
%
% function [f, v, c] = GetLowerLegPatch()
% 
% Returns faces, vertices, and color-values (one per face) that represent a
% patch of the lower segment of a prismatic leg.
%
% - The foot center is at [0, 0, 0]
% - The leg points into direction [0, 0, 1]
% - Foot radius is 0.05;
% - The lower leg spring should start at [0, 0, 0.16]
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
function [f,v,c] = GetLowerLegPatch()

    % Spherical foot with radius 0.05.  The center of this segment will be
    % the center of the foot-sphere
    [x y z] = sphere; 
    [f, v] = surf2patch(x, y, z, z);
    % Scale to diameter 0.1
    v = v * 0.1;
    % Set a uniform black color:
    c = ones(length(f), 3)*0;
    % Lower leg segment (length 0.6 (from foot center to top)
    % The spring starts at 0.16 (from foot center)
    % Create a cylinder with the shape of the lower foot segment
    [x, y] = cylinder([0.031, 0.031, 0.1-0.069*(cos((0:1:10)/10*pi/2)), 0.1 , 0.1, 0.1, 0.031, 0.031, 0.031, 0.031, 0]);
    z = repmat([0; 0.1; 0.1+0.05*(sin((0:1:10)/10*pi/2))'; 0.15; 0.16; 0.16; 0.16; 0.16; 0.6; 0.6; 0.6], 1, 21);
    [f1, v1] = surf2patch(x, y, z, z);
    % Set a uniform light grey color
    c1 = ones(length(f1), 3)*0.95;
    % Color two segments black (where the spring rests)
    c1(14:20:400, :) = repmat([0 0 0], 20, 1);
    c1(16:20:400, :) = repmat([0 0 0], 20, 1);
    
    % Join foot and lower leg segment:
    [v, f, c] = AddPatchesWithColor(v, f, c, v1, f1, c1);
end
% *************************************************************************
% *************************************************************************