% *************************************************************************
%
%   function [X, Y, Z] = GetSpringSurface(N, M, L, R, r, n) 
%   function [X, Y, Z] = GetSpringSurface() 
% 
%  Generates three (N+1)-by-(M+1) matrices such that SURF(X, Y, Z) produces
%  a spring coil. 
%
% Input:  - N [20] subdevision around the coil
%         - M [100] subdevision along the coil
%         - L [3] length of the spring
%         - R [1] radius of the coil
%         - r [0.1] radius of the wire
%         - n [5] number of  windings (must be an odd number)
% 
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
function [xOUT, yOUT, zOUT] = GetSpringSurface(varargin)
    % Check if input was provided
    if nargin<6
        N = 20;
        M = 100;
        L = 3;
        R = 1;
        r = 0.1;
        n = 5;
    else
        N = varargin{1};
        M = varargin{2};
        L = varargin{3};
        R = varargin{4};
        r = varargin{5};
        n = varargin{6};
    end

    % -pi <= theta <= pi is a row vector.
    % -pi <= phi <= pi is a column vector.
    theta = (-M:2:M)/M*pi*n;  % coil rotation
    phi   = (-N:2:N)'/N*pi;   % wire rotation
    l = (0:1:M)/M*(L+2*r)-r;    % along the coil-axis
    sinphi = sin(phi); sinphi(1) =  0; sinphi(N+1) =  0;
    cosphi = cos(phi); cosphi(1) = -1; cosphi(N+1) = -1;
    sintheta = sin(theta); sintheta(1)   =  0; sintheta(M+1) =  0;
    costheta = cos(theta); costheta(1)   = -1; costheta(M+1) = -1;
    
    x = cosphi*costheta*r+ones(N+1,1)*costheta*R;
    y = cosphi*sintheta*r+ones(N+1,1)*sintheta*R;
    z = sinphi*ones(1,M+1)*r + ones(N+1,1)*l;
    z = min(z,L);
    z = max(z,0);
    
    if nargout == 0
        surf(x, y, z)
    else
        xOUT = x; yOUT = y; zOUT = z;
    end
end
% *************************************************************************
% *************************************************************************
