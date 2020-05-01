% *************************************************************************
%
% function [f, v, c] = Get3DGroundPatch()
% 
% Returns faces, vertices, and color-values (one per face) that represent a
% patch of the ground floor.  This is checkerboard at level 0,reaching from
% -5 to +5.
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
function [f, v, c] = Get3DGroundPatch()
    % Create the ground. It reaches from -5 to +5
    v = []; f = []; c = [];
    % Number of tiles on each side of the origin
    nTilesX = 5;  
    nTilesZ = 5;  
    % Size of the tiles
    sTiles = 1;  
    % Create vertices:
    for i = -sTiles*nTilesX:sTiles:sTiles*nTilesX
        for j = -sTiles*nTilesZ:sTiles:sTiles*nTilesZ
            v = [v;[i,j,0]];                         
        end
    end
    % Connect them and color them as checkerboard:
    for i = 1:2*nTilesZ
        for j = 1:2*nTilesX
            f = [f;[i,i+1,i+2+2*nTilesZ,i+1+2*nTilesZ]+(j-1)*(2*nTilesZ+1)];	% Connect vertices
            if mod(j+i,2)==0;	% Color faces alternating
                c = [c;[1,1,1]];
            else
                c = [c;[0,0,0]];
            end
        end
    end
end
% *************************************************************************
% *************************************************************************