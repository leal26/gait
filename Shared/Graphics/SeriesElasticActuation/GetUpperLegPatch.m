% *************************************************************************
%
% function [f, v, c] = GetUpperLegPatch()
% 
% Returns faces, vertices, and color-values (one per face) that represent a
% patch of the upper segment of a prismatic leg.
%
% - The hip joint is at [0,0,0]
% - The leg points into direction [0,0,-1]
% - The radius of the upper actuator is 0.2427 (cos(atan2(0.1,0.26))*0.26)
% - The face on which the spring lies has an angle of 21.0375
%   (atan2(0.1,0.26)) with the vertical
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
function [f, v, c] = GetUpperLegPatch()
    
    % Main cylinder along the prismatic axis:
    [x, y] = cylinder([0, 0.037, 0.037, 0.037, 0.037, 0]);
    z = repmat([-0.6, -0.6, -0.6, 0, 0, 0]', 1, 21);
    [f, v] = surf2patch(x, y, z, z);
    % Set a uniform medium gray color:
    c = ones(length(f), 3)*0.5;
    % Main Element (a cylinder that is deformed manually)
    [x,y] = cylinder([0.04, 0.04, 0.04, 0.1, 0.1, 0.1, 0.1, 0], 26);
    z = repmat([-0.2, -0.1, -0.1, -0.1, -0.1, 0.1, 0.1, 0.1]', 1, 27);
    s = 0.16*0.1/0.35; % defines where the lateral actuator will be mounted.
    x(4:7, 15:27) = repmat([-0.1, -0.1, -0.1+s, -0.1+s, -0.1-s, -0.1-s, -0.1, -0.1, -0.1, -0.1, 0.1, 0.1, 0.1], 4, 1);
    y(4:7, 15:27) = repmat([-0.08, -0.08, -0.13, -0.13, -0.33, -0.33, -0.34, -0.34, -0.35, -0.35, -0.35, -0.35, 0], 4, 1);
    y(8, :) = repmat(-0.1, 1, 27);
    [f1, v1] = surf2patch(x,y,z,z);
    % Set a uniform grey color
    c1 = ones(length(f1), 3)*0.5;
    % color one segment (the main axis) black
    c1(1:7:182, :) = repmat([0 0 0], 26, 1);
    v1 = TransformVertices(v1, [1,0,0;0,0,1;0,1,0], [0,0,0]);
    % Join with the prismatic axis
    [v, f, c] = AddPatchesWithColor(v, f, c, v1, f1, c1);
    
    % Bent rotational cycliner
    [x,y] = cylinder([0;ones(20,1)*0.031;0], 20);
    z = repmat([-1, -1, linspace(-1,0,18), 0, 0]', 1, 21);
    [f1, v1] = surf2patch(x, y, z, z);
    % Set a uniform grey color
    c1 = ones(length(f1), 3)*0.5;
    v1 = TransformVertices(v1, eye(3),[cos(atan2(0.1,0.26))*0.26, 0, -5/4]);
    v1 = BendVertices(v1, 5);
    % Join with the rest
    [v, f, c] = AddPatchesWithColor(v, f, c, v1, f1, c1);
end
% *************************************************************************
% *************************************************************************