% *************************************************************************
%
% function [f, v, c] = GetETHLogoPatch()
% 
% Returns faces, vertices, and color-values (one per face) that represent a
% patch of the ETH-Logo.  
% 
% The logo has height 1, width 1, and length 3.0134
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
%   See also GETMAINBODYMONOPEDPATCH, PATCH.
%
function [f, v, c] = GetETHLogoPatch()
    % Definition of polyline (As defined in CAD):
    ETHlines = ... 
    ([37.3941,25.2158;
    37.3941,25.2158;
    36.6882,22.2563;
    36.6882,22.2563;
    38.5887,22.2563;
    38.5887,22.2563;
    39.2946,25.2158;
    39.2946,25.2158;
    42.0368,25.2158;
    42.0368,25.2158;
    40.2449,17.1524;
    40.2449,17.1524;
    37.5303,17.1524;
    37.5303,17.1524;
    38.1271,20.2475;
    38.1271,20.2475;
    36.3352,20.2475;
    36.3352,20.2475;
    35.52070000000001,17.1524;
    35.52070000000001,17.1524;
    32.67010000000001,17.1524;
    32.67010000000001,17.1524;
    33.97340000000001,23.0708;
    33.97340000000001,23.0708;
    31.4759,23.0708;
    31.4759,23.0708;
    30.1724,17.1524;
    30.1724,17.1524;
    27.2131,17.1524;
    27.2131,17.1524;
    28.516,23.0708;
    28.516,23.0708;
    22.1367,23.0708;
    22.1367,23.0708;
    21.892,22.1207;
    21.892,22.1207;
    25.91,22.1207;
    25.91,22.1207;
    25.4487,20.2475;
    25.4487,20.2475;
    21.4302,20.2475;
    21.4302,20.2475;
    21.1864,19.2974;
    21.1864,19.2974;
    25.3398,19.2974;
    25.3398,19.2974;
    24.8512,17.1524;
    24.8512,17.1524;
    17.7383,17.1524;
    17.7383,17.1524;
    19.5303,25.2158;
    19.5303,25.2158;
    37.3941,25.2158;
    37.3941,25.2158] - repmat([17.7383,17.1524],54,1))*diag([1/(25.2158-17.1524),1/(25.2158-17.1524)]);
    % Define vertices and faces:
    v = [[ETHlines; ETHlines; ETHlines; ETHlines],[zeros(54,1); zeros(54,1); ones(54,1); ones(54,1)]];
    f = [fliplr([3,4,5,6;
         3,6,7,8;
         3,8,9,2;
         9,10,11,12;
         9,12,1,2;
         12,13,26,27;
         13,14,15,16;
         16,17,26,12;
         17,18,25,26;
         18,19,20,21;
         18,21,22,25;
         22,23,24,25])*2;
         repmat([1,2,54+2,54+1]+55,26,1)+repmat([0:2:50]',1,4);
         fliplr([3,4,5,6;
         3,6,7,8;
         3,8,9,2;
         9,10,11,12;
         9,12,1,2;
         12,13,26,27;
         13,14,15,16;
         16,17,26,12;
         17,18,25,26;
         18,19,20,21;
         18,21,22,25;
         22,23,24,25])*2+3*54];
	% Color is black
    c = ones(length(f),3)*0.0;
end
% *************************************************************************
% *************************************************************************