% *************************************************************************
%
% function vTrans = BendVertices(v, ratio)
% 
% This function transforms the coordinates of the vertices given in 'v'.
% They are 'bendt' around the origins y axis according to their z value.
% 'ratio' describes what z-value should correspond to what angle.  Or, more
% precise the z-value that corresponds to 360 degresse: 
% xNew = x*cos(z*2*pi/ratio);
% yNew = y;
% zNew = x*sin(z*2*pi/ratio);
% 
%
% Input:  - A nx3 matrix of vertices 'v' stored in rows
%         - a double 'ratio' that specifies how mucht the vertices are bend
%           around the y axis
%         
% Output: - A nx3 matrix of transformed vertices.
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
%   See also TRANSFORMVERTICES, PATCH. 
%
function vTrans = BendVertices(v, ratio)
    if isempty(v)
        vTrans = [];
        return
    end
    % Create vectors for each coordinates
    x = v(:,1);
    y = v(:,2);
    z = v(:,3);
    % Transform according to the z-value
    xNew = x.*cos(z*2*pi/ratio);
    yNew = y;
    zNew = x.*sin(z*2*pi/ratio);
    % Compose back into a single matrix
    vTrans = [xNew, yNew, zNew];
end
% *************************************************************************
% *************************************************************************