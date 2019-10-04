% *************************************************************************
%
% function [f, v, c] = 
%  GetQuadrupedPatch(x, y, phi, alphaF, ualphaF, alphaB, ualphaB, lF, ulF, lB, ulB)
% Returns faces, vertices, and color-values representing a patch that shows
% the 3D view of a bounding quadruped with two degrees of freedom.
%
%  GetQuadrupedPatch(x, y, phi, alphaF, ualphaF, alphaB, ualphaB, lF, ulF,
%  lB, ulB), without return arguments shows the patch in a new figure
%
% Input:  - x, y, phi (position and orientation of the main body)
%         - alphaF(B), lF(B) (length and orientation of the front (back)
%             leg) 
%         - ualphaF(B), ulF(B) (deflection of the front (back) actuators
%             from their resting position)
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
function [f,v,c] = GetQuadrupedPatch(x, y, phi, alphaF, ualphaF, alphaB, ualphaB, lF, ulF, lB, ulB)

    % The faces, vertices, and color values are only created once and then
    % stored in memory:
    persistent mainBodyVertices upperLegVertices lowerLegVertices upperSpringVertices lowerSpringVertices upperActuatorVertices lowerActuatorVertices
    persistent f1 f2 f3 f4 f5 f6 f7
    persistent c1 c2 c3 c4 c5 c6 c7
    if isempty(mainBodyVertices) || isempty(upperLegVertices) || isempty(lowerLegVertices) || isempty(upperSpringVertices) || isempty(lowerSpringVertices) || isempty(upperActuatorVertices) || isempty(lowerActuatorVertices)
        [f1,mainBodyVertices,c1]     = GetMainBodyQuadrupedPatch();
        [f2,upperLegVertices,c2]      = GetUpperLegPatch();
        [f3,lowerLegVertices,c3]      = GetLowerLegPatch();
        [f4,upperSpringVertices,c4]   = GetUpperSpringPatch();
        [f5,lowerSpringVertices,c5]   = GetLowerSpringPatch();
        [f6,upperActuatorVertices,c6] = GetUpperActuatorPatch();
        [f7,lowerActuatorVertices,c7] = GetLowerActuatorPatch();
    end
    
    % NOTE: Everything is implicitly transformed from the [X Y Z] in which
    % the dynamics are stated into the [X -Z Y] system in which the
    % graphics are displayed, i.e. the translation of the main body is
    % [x,0,y] instead of [x,y,0]. The main body and leg angles are thus
    % inverted: 
    phi     = -phi;
    alphaF  = -alphaF;
    ualphaF = -ualphaF;
    alphaB  = -alphaB;
    ualphaB = -ualphaB;
           
    %% Transform the vertices, and 'add' up the final patch object
    %% Main body:
    v = TransformVertices(mainBodyVertices, Body312dc([0,0,phi]), [x,0,y]);f = f1;c = c1;
    %% Upper leg segments (rotate and shift side wards)
    v_ = TransformVertices(upperLegVertices,Body312dc([0,0,alphaF]),[0,0,0]); % rotate
    v2 = TransformVertices(v_,diag([+1,+1,1]), [+0.75,+0.6,0]); % shift within main body
    v2 = TransformVertices(v2,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v2,f2,c2);
    v2 = TransformVertices(v_,diag([+1,-1,1]), [+0.75,-0.6,0]); % shift within main body
    v2 = TransformVertices(v2,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v2,f2,c2);
    v_ = TransformVertices(upperLegVertices,Body312dc([0,0,-alphaB]),[0,0,0]); % rotate
    v2 = TransformVertices(v_,diag([-1,+1,1]), [-0.75,+0.6,0]); % shift within main body
    v2 = TransformVertices(v2,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v2,f2,c2);
    v2 = TransformVertices(v_,diag([-1,-1,1]), [-0.75,-0.6,0]); % shift within main body
    v2 = TransformVertices(v2,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v2,f2,c2);
    %% Lower leg segments (move downwards, rotate and shift side wards)
    v_ = TransformVertices(lowerLegVertices,eye(3),[0,0,-lF]);
    v_ = TransformVertices(v_,Body312dc([0,0,alphaF]),[0,0,0]); % rotate
    v3 = TransformVertices(v_,diag([+1,+1,1]), [+0.75,+0.6,0]); % shift within main body
    v3 = TransformVertices(v3,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v3,f3,c3);
    v3 = TransformVertices(v_,diag([+1,-1,1]), [+0.75,-0.6,0]); % shift within main body
    v3 = TransformVertices(v3,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v3,f3,c3);
    v_ = TransformVertices(lowerLegVertices,eye(3),[0,0,-lB]);
    v_ = TransformVertices(v_,Body312dc([0,0,-alphaB]),[0,0,0]); % rotate
    v3 = TransformVertices(v_,diag([-1,+1,1]), [-0.75,+0.6,0]);  % shift within main body
    v3 = TransformVertices(v3,Body312dc([0,0,phi]), [x,0,y]);    % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v3,f3,c3);
    v3 = TransformVertices(v_,diag([-1,-1,1]), [-0.75,-0.6,0]);  % shift within main body
    v3 = TransformVertices(v3,Body312dc([0,0,phi]), [x,0,y]);    % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v3,f3,c3);
    %% Lower actuators (move downwards, rotate and shift side wards)
    v_ = TransformVertices(lowerActuatorVertices,eye(3),[0,0,(-0.5 -ulF)]);
    v_ = TransformVertices(v_,Body312dc([0,0,alphaF]),[0,0,0]); % rotate
    v6 = TransformVertices(v_,diag([+1,+1,1]), [+0.75,+0.6,0]); % shift within main body
    v6 = TransformVertices(v6,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v6,f6,c6);
    v6 = TransformVertices(v_,diag([+1,-1,1]), [+0.75,-0.6,0]); % shift within main body
    v6 = TransformVertices(v6,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v6,f6,c6);              
    v_ = TransformVertices(lowerActuatorVertices,eye(3),[0,0,(-0.5 -ulB)]);
    v_ = TransformVertices(v_,Body312dc([0,0,-alphaB]),[0,0,0]); % rotate
    v6 = TransformVertices(v_,diag([-1,+1,1]), [-0.75,+0.6,0]);  % shift within main body
    v6 = TransformVertices(v6,Body312dc([0,0,phi]), [x,0,y]);    % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v6,f6,c6);
    v6 = TransformVertices(v_,diag([-1,-1,1]), [-0.75,-0.6,0]);  % shift within main body
    v6 = TransformVertices(v6,Body312dc([0,0,phi]), [x,0,y]);    % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v6,f6,c6);              
    %% Lower spring (scale, move downwards, rotate and shift side wards)
    delta_lF = (1-(1+ulF-lF)/0.34);
    v_ = TransformVertices(lowerSpringVertices,diag([1,1,delta_lF]),[0,0,(-lF + 0.16)]);
    v_ = TransformVertices(v_,Body312dc([0,0,alphaF]),[0,0,0]); % rotate
    v5 = TransformVertices(v_,diag([+1,+1,1]), [+0.75,+0.6,0]); % shift within main body
    v5 = TransformVertices(v5,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v5,f5,c5);
    v5 = TransformVertices(v_,diag([+1,-1,1]), [+0.75,-0.6,0]); % shift within main body
    v5 = TransformVertices(v5,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v5,f5,c5);
    delta_lB = (1-(1+ulB-lB)/0.34);
    v_ = TransformVertices(lowerSpringVertices,diag([1,1,delta_lB]),[0,0,(-lB + 0.16)]);
    v_ = TransformVertices(v_,Body312dc([0,0,-alphaB]),[0,0,0]); % rotate
    v5 = TransformVertices(v_,diag([-1,+1,1]), [-0.75,+0.6,0]);  % shift within main body
    v5 = TransformVertices(v5,Body312dc([0,0,phi]), [x,0,y]);    % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v5,f5,c5);
    v5 = TransformVertices(v_,diag([-1,-1,1]), [-0.75,-0.6,0]);  % shift within main body
    v5 = TransformVertices(v5,Body312dc([0,0,phi]), [x,0,y]);    % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v5,f5,c5);
    %% Upper actuators (rotate and shift side wards)
    v_ = TransformVertices(upperActuatorVertices,Body312dc([0,0,ualphaF]),[0,0,0]); % rotate
    v7 = TransformVertices(v_,diag([+1,+1,1]), [+0.75,+0.6,0]); % shift within main body
    v7 = TransformVertices(v7,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v7,f7,c7);
    v7 = TransformVertices(v_,diag([+1,-1,1]), [+0.75,-0.6,0]); % shift within main body
    v7 = TransformVertices(v7,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v7,f7,c7);              
    v_ = TransformVertices(upperActuatorVertices,Body312dc([0,0,-ualphaB]),[0,0,0]); % rotate
    v7 = TransformVertices(v_,diag([-1,+1,1]), [-0.75,+0.6,0]); % shift within main body
    v7 = TransformVertices(v7,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v7,f7,c7);
    v7 = TransformVertices(v_,diag([-1,-1,1]), [-0.75,-0.6,0]); % shift within main body
    v7 = TransformVertices(v7,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v7,f7,c7);              
    %% Upper springs (bent& compress, rotate and shift side wards)
    delta_alphaF = atan2(0.26,0.1) + (ualphaF-alphaF);
    % Bend including compression:
    v_ = BendVertices(upperSpringVertices,0.25/(delta_alphaF)*2*pi);
    v_ = TransformVertices(v_,Body312dc([0,0,ualphaF]),[0,0,0]); % rotate
    v4 = TransformVertices(v_,diag([+1,+1,1]), [+0.75,+0.6,0]); % shift within main body
    v4 = TransformVertices(v4,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v4,f4,c4);
    v4 = TransformVertices(v_,diag([+1,-1,1]), [+0.75,-0.6,0]); % shift within main body
    v4 = TransformVertices(v4,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v4,f4,c4);
    delta_alphaB = atan2(0.26,0.1) - (ualphaB-alphaB);
    % Bend including compression:
    v_ = BendVertices(upperSpringVertices,0.25/(delta_alphaB)*2*pi);
    v_ = TransformVertices(v_,Body312dc([0,0,-ualphaB]),[0,0,0]); % rotate
    v4 = TransformVertices(v_,diag([-1,+1,1]), [-0.75,+0.6,0]); % shift within main body
    v4 = TransformVertices(v4,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v4,f4,c4);
    v4 = TransformVertices(v_,diag([-1,-1,1]), [-0.75,-0.6,0]); % shift within main body
    v4 = TransformVertices(v4,Body312dc([0,0,phi]), [x,0,y]);   % shift with main body
    [v,f,c]=AddPatchesWithColor(v,f,c,v4,f4,c4);
    
    if nargout == 0
        figure;
        p = patch('faces', f, 'vertices' ,v,'FaceVertexCData',c,'FaceColor','flat');
        view(3);
        axis equal;
        set(p, 'FaceLighting','phong');                                            % Set the renderer
        set(p, 'FaceColor','flat');                                                % Set the face coloring
        set(p, 'EdgeColor','none'); 
        camlight right
    end
end
% *************************************************************************
% *************************************************************************