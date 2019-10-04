% *************************************************************************
%
% function R = Body312dc(ang)
% 
% Computes the direction cosine relating the distal segment
% to the proximal segment following a body fixed 3-1-2 rotation
% sequence of ang(1), ang(2), ang(3)
% 
%
% Input:  - a 1x3 vector of rotation angles 'ang'
%         
% Output: - a 3x3 rotation matrix 'R'
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
%   See also PATCH. 
%
function R = Body312dc(ang)
    % Compute sine and cosine values beforehand
    s1 = sin(ang(1));  c1 = cos(ang(1));
    s2 = sin(ang(2));  c2 = cos(ang(2));
    s3 = sin(ang(3));  c3 = cos(ang(3));
    % Compose rotation matrix
    R=[-s1*s2*s3+c1*c3   -s1*c2            s1*s2*c3+c1*s3
        c1*s2*s3+s1*c3    c1*c2            -c1*s2*c3+s1*s3
        -c2*s3            s2                c2*c3];
end
% *************************************************************************
% *************************************************************************