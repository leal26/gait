% *************************************************************************
%
% function [f, v, c] = Get2DGroundPatch()
% 
% Returns faces, vertices, and color-values (one per face) that represent a
% patch of the ground floor.  This is essentially a shaded area at level 0,
% reaching from -5 to +5
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
%   See also OUTPUTCLASS, PATCH.
%
function [f, v, c] = Get2DGroundPatch()
    % Create the ground. It reaches from -5 to +5.
    h   = 0.05; % Height of the bar at the top
    n   = 100;  % Number of diagonal stripes in the shaded area
    s   = 0.1;  % Spacing of the stripes
    w   = 0.04; % Width of the stripes
    ext = 0.2;  % Length of the stripes
    % Create vertices by shifting a predefined pattern 'n' times to the
    % right:
    v = [-5,0,0;
         repmat([0,0,-h],n,1) + [-5+linspace(0,s*n,n)',zeros(n,2)];
         repmat([-ext,0,-ext-h],n,1) + [-5+linspace(0,s*n,n)',zeros(n,2)];
         repmat([-ext+w,0,-ext-h],n,1) + [-5+linspace(0,s*n,n)',zeros(n,2)];
         repmat([w,0,0-h],n,1) + [-5+linspace(0,s*n,n)',zeros(n,2)];
         -5+s*n+w,0,0];
    % Connect to faces:
    f = [1,2,4*n+1,4*n+2;
         repmat([0,n,2*n,3*n],n,1) + repmat((1:n)',1,4)+1];
	% Color is uniformly black
    c = ones(length(f),3)*0.2;
end
% *************************************************************************
% *************************************************************************