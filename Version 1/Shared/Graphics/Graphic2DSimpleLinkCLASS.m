% *************************************************************************
%
% classdef Graphic2DSimpleLinkCLASS(y, p, rFoot, rCoG) < OutputCLASS
%
% A link (segment) and CoG based graphical representation of the dynamic
% model.  On the MATLAB search path, an implementation of the following
% function must be found:
%
% [CoGs, links, footPts] = GraphicalKinematicsWrapper(y, p)
% 
% which maps a vector of continuous states and system parameters into
% vectors of CoG positions and orientations, link positions, and foot point
% positions.  The class then draws:
%
%  - A ground surface representation
%  - A CoG Symbol at every row of the 'CoG'-vector
%  - A line object connecting all points given in 'links' in red
%  - Circular feet at all points indicated by 'footPts' in blue
%
%  This class is created by calling the constructor 
%  obj = Graphic2DSimpleLinkCLASS(y, p, rFoot) with the following inputs:
%         - A vector of continuous states 'y' 
%         - A vector of model system parameters 'p'
%         - The radius 'rFoot' with which the feet should be drawn.  
%         - A vector of radii 'rCoG' for the CoG symbols
%
%
% Methods:    - NONE
%
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
%   See also OUTPUTCLASS, PLOTSTATECLASS, RECORDSTATECLASS. 
%
classdef Graphic2DSimpleLinkCLASS < OutputCLASS 
    properties (SetAccess = 'private', GetAccess = 'private')
        fig; % The output window
        ax;  % The output axis
        % Parameters
        rFoot;
        rCoG;
        p;
        % Graphical objects (and the respective data) used in the 2D
        % representation:
        xdata_ground;
        ydata_ground;
        ground_graph;
        CoGPatches;
        linkLines;
        FootPatches;
        hText;
    end
    methods
        function obj = Graphic2DSimpleLinkCLASS(y, p, rFoot, rCoG)
            % Set up object parameters:
            obj.slowDown = 0.5;   % Run this a bit faster than realtime
            obj.rate     = 0.01;  % With a high resolution
            obj.rFoot    = rFoot; % Save the foot radius for later use
            obj.rCoG     = rCoG;  % Save the CoG radii for later use
            obj.p        = p;     % Save the parameter vector for later use
            % Set up output figure:
            obj.fig = figure;
            obj.ax = axes;
            hold on
            box on
            grid on
            axis equal
            set(obj.fig,'Name','2D-Output using a simplified link-based representation');
            set(obj.fig,'Color','w');
            % Get initial values, which determine how many links, CoGs,
            % etc. we will draw:
            [CoGs, links, footPts] = GraphicalKinematicsWrapper(y, p);
            % Create the ground. The shaded area reaches from +-5. It has
            % to be shifted in the plot routine if the quadruped is moving.
            % Only the Baseline is 'infinitely' long
            line([-10000000,10000000],[0,0],'LineWidth',3,'Color',[0,0,0]);
            % The ground:
            obj.xdata_ground=[];
            obj.ydata_ground=[];
            for i = -15:0.15:15
                obj.xdata_ground = [obj.xdata_ground,[i-0.15;i+0.15]];
                obj.ydata_ground = [obj.ydata_ground,[-0.3;0]];
            end
            obj.ground_graph = line(obj.xdata_ground,obj.ydata_ground,'Color',[0,0,0]);
            % The CoGs, links, feet
            obj.FootPatches = [];
            for i = 1:size(footPts,2)
                [~, ~, patchH] = obj.GetFootPatch(footPts(1,i), footPts(2,i), obj.rFoot);
                obj.FootPatches = [obj.FootPatches, patchH];
            end
            obj.linkLines = line(links(1,:),links(2,:),'Color',[1,0,0],'LineWidth',2,'Marker','.','MarkerSize',20);
            obj.CoGPatches = [];
            for i = 1:size(CoGs,2)
                [~, ~, patchH] = obj.GetCoGPatch(CoGs(1,i), CoGs(2,i), CoGs(3,i), obj.rCoG(i));
                obj.CoGPatches = [obj.CoGPatches, patchH];
            end
            % Crop plot to reasonable dimensions
            axis(obj.ax,[floor(min(CoGs(1,:))-0.5), ceil(max(CoGs(1,:))+0.5), -1, ceil(max(CoGs(2,:))+0.5)]);
            % Display the simulation time
            obj.hText = text(floor(min(CoGs(1,:))-0.5)+0.2,ceil(max(CoGs(2,:))+0.5)-0.2,['Time: ',num2str(0)]);
        end
        % Implementation of the abstract super class:
        function obj = update(obj, y, ~, t, ~)
            % Plot the current state of the system by updating the lines
            % and patches that have been saved before 
            
            % Get an updated computation of the kinematics:
            [CoGs, links, footPts] = GraphicalKinematicsWrapper(y, obj.p);
            % Update the CoGs, links, feet:
            for i = 1:size(CoGs,2)
                [xData, yData] = obj.GetCoGPatch(CoGs(1,i), CoGs(2,i), CoGs(3,i), obj.rCoG(i));
                set(obj.CoGPatches(i),'XData',xData,'YData',yData);
            end
            for i = 1:size(footPts,2)
                [xData, yData] = obj.GetFootPatch(footPts(1,i), footPts(2,i),  obj.rFoot);
                set(obj.FootPatches(i),'XData',xData,'YData',yData);
            end
            set(obj.linkLines,'XData',links(1,:),'YData',links(2,:));
            % Shift the ground to the next integer next to the center of
            % the model:
            for m = 1:length(obj.xdata_ground)
               set(obj.ground_graph(m),'XData',obj.xdata_ground(:,m)+round(CoGs(1,1)));
            end
            % The axis is set, such that it overlaps all CoGs by about 2:
            if isnan(ceil(max(CoGs(2,:)))+1)
                disp('ala');
            end
            axis(obj.ax,[floor(min(CoGs(1,:))-0.5), ceil(max(CoGs(1,:))+0.5), -1, ceil(max(CoGs(2,:))+0.5)]);
            set(obj.hText,'Position',[floor(min(CoGs(1,:))-0.5)+0.2,ceil(max(CoGs(2,:))+0.5)-0.2,0],'String',['Time: ',num2str(t)]);
            drawnow();
        end
    end
    methods(Static)
        % These functions return patch objects for the CoG and foot
        % representation:
        function [xData, yData, patchH] = GetCoGPatch(x, y, ang, r)
            n_CoG = linspace(0,2*pi,21);
            xPos = x + cos(n_CoG+ang)*r;
            yPos = y + sin(n_CoG+ang)*r;
            xData = [x,x,x,x;xPos(1:6)',xPos(6:11)',xPos(11:16)',xPos(16:21)'];
            yData = [y,y,y,y;yPos(1:6)',yPos(6:11)',yPos(11:16)',yPos(16:21)'];
            cData(1,:,:) = [0 0 0; 1 1 1; 0 0 0; 1 1 1];
            if nargout>2
                patchH = patch(xData, yData, cData);
            end
        end
        function [xData, yData, patchH] = GetFootPatch(x, y, r)
            n_Foot = linspace(0,2*pi,21);
            xData = x + cos(n_Foot)*r;
            yData = y + sin(n_Foot)*r;
            cData(1,1,:) = [0 0 1];
            if nargout>2
                patchH = patch(xData, yData, cData);
            end
        end
    end
end
% *************************************************************************
% *************************************************************************