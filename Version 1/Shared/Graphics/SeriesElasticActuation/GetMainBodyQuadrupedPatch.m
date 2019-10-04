% *************************************************************************
%
% function [f, v, c] = GetMainBodyQuadrupedPatch()
% 
% Returns faces, vertices, and color-values (one per face) that represent a
% patch of the main body of a series elastic quadruped
%
% - The legs must be position at [0.75,0.6,0], [0.75,-0.6,0],
%   [-0.75,0.6,0], and [-0.75,-0.6,0]  on the sides of the robot 
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
%   See also GETQUADRUPEDPATCH, PATCH.
%

function [f,v,c] = GetMainBodyQuadrupedPatch()

    % The main body is composed as a box (which is first defined for an
    % extension of +/-1 and scaled later).  Each corner exists three times,
    % such that the normal vectors for each face are independent
    v = [-1,-1,-1;
         -1,-1,-1;
         -1,-1,-1;
         -1,+1,-1;
         -1,+1,-1;
         -1,+1,-1;
         -1,+1,+1;
         -1,+1,+1;
         -1,+1,+1;
         -1,-1,+1;
         -1,-1,+1;
         -1,-1,+1;
         +1,-1,-1;
         +1,-1,-1;
         +1,-1,-1;
         +1,+1,-1;
         +1,+1,-1;
         +1,+1,-1;
         +1,+1,+1;
         +1,+1,+1;
         +1,+1,+1;
         +1,-1,+1;
         +1,-1,+1;
         +1,-1,+1];
    % Define faces of a box (note that the vertices have been defined individually for each segment, to for
    f = [10,7,4,1;
         13,16,19,22;
         5,8,20,17;
         2,14,23,11;
         9,12,24,21;
         3,6,18,15]; 
    % Scale to main body dimensions of [1.9 x 0.9 x 0.4]:
    v =  TransformVertices(v,diag([0.95,0.45,0.2]),[0,0,0]);                     % Transformation into local system
    % Set a uniform grey color:
    c = ones(length(f),3)*diag([0.8, 0.8, 0.8]);
    
	% Bent rotational cylinder (on which the hip actuation is attached) on
	% both sides: 
    [x, y] = cylinder([0;ones(40,1)*0.037;0],20);
    z = repmat([-1.2, -1.2, linspace(-1,0,38), 0, 0]',1,21);
    [f1, v1] = surf2patch(x,y,z,z);
    % Set a uniform dark grey color
    c1 = ones(length(f1),3)*0.2;
    % Move and add it on the first side of the robot...
    v1 = TransformVertices(v1,eye(3),[cos(atan2(0.1,0.26))*0.26,0,0]);
    v1 = BendVertices(v1,8);
    v2 = TransformVertices(v1,eye(3),[-0.75,0.6,0]);
    [v,f,c]=AddPatchesWithColor(v,f,c,v2,f1,c1);
    % ... and on the other side:
    v2 = TransformVertices(v1,eye(3),[-0.75,-0.6,0]);
    [v,f,c]=AddPatchesWithColor(v,f,c,v2,f1,c1);
    % ... and on the front:
    v2 = TransformVertices(v1,diag([-1,1,1]),[0.75,0.6,0]);
    [v,f,c]=AddPatchesWithColor(v,f,c,v2,f1,c1);
    % ... and on the other side of the front:
    v2 = TransformVertices(v1,diag([-1,1,1]),[0.75,-0.6,0]);
    [v,f,c]=AddPatchesWithColor(v,f,c,v2,f1,c1);
    
    % Add the ETH/ASL Logo on both sides:
    [f1,v1,c1] = GetASLLogoPatch();
    % Scale and move
    v1 = TransformVertices(v1,[-1,0,0;0,0,1;0,1,0]*0.16,[0.15,0.30,0.02]);
    [v,f,c]=AddPatchesWithColor(v,f,c,v1,f1,c1);
    [f1,v1,c1] = GetETHLogoPatch();
    % Scale and move
    v1 = TransformVertices(v1,[-1,0,0;0,0,1;0,1,0]*0.1,[0.0,0.36,-0.12]);
    [v,f,c]=AddPatchesWithColor(v,f,c,v1,f1,c1);
    % And on the other side:
    [f1,v1,c1] = GetASLLogoPatch();
    % Scale and move
    v1 = TransformVertices(v1,[1,0,0;0,0,-1;0,1,0]*0.16,[-0.15,-0.30,0.02]);
    [v,f,c]=AddPatchesWithColor(v,f,c,v1,f1,c1);
    [f1,v1,c1] = GetETHLogoPatch();
    % Scale and move
    v1 = TransformVertices(v1,[1,0,0;0,0,-1;0,1,0]*0.1,[-0.0,-0.36,-0.12]);
    [v,f,c]=AddPatchesWithColor(v,f,c,v1,f1,c1);
end
% *************************************************************************
% *************************************************************************