% *************************************************************************
%
% function [v, f, c] = AddPatchesWithColor(v1, f1, c1, v2, f2, c2)
% 
% Assuming that v1, f1, c1, v2, f2, and c2 define vertices, faces, and
% colors of two patch objects, this function returns the vertices, faces,
% and colors of a new combined patch object.
% 
%
% Input:  - vertices 'v1' and 'v2' of two patch objects
%         - faces 'f1' and 'f2' of two patch objects
%         - colors 'c1' and 'c2' of two patch objects
%         
% Output: - vertices 'v' of the combined patch object
%         - faces 'f' of the combined patch object
%         - colors 'c' of the combined patch object
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
function [v, f, c] = AddPatchesWithColor(v1, f1, c1, v2, f2, c2)
    % The indices of the second matrix of faces are increased...
    f2 = f2 + repmat(size(v1,1), size(f2));
    % ... and the matrices simply composed
    v = [v1; v2];
    f = [f1; f2];
    c = [c1; c2];
end
% *************************************************************************
% *************************************************************************