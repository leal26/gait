% *************************************************************************
% classdef SLIP_Model_Graphics(p) < OutputCLASS
%
% Two dimensional graphics of a SLIP model.
%
% The graphics object must be initialized with the vector of system
% parameters p.
%
%
% Properties: - NONE
% Methods:    - NONE
%
%
% Created by C. David Remy on 07/10/2011
% MATLAB 2010a - Windows - 64 bit
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
%   See also OUTPUTCLASS.
%
classdef SLIP_Model_Graphics_PointFeet < OutputCLASS 
    % Private attributes:
    properties 
        % The output window
        fig; 
        % Patch and line objects used in the graphical representation
        COGPatch;
        Body;
        SpringLine_l;
        SpringLine_r;

    end
    % Public methods:
    methods
        % Constructor:
        function obj = SLIP_Model_Graphics_PointFeet
            obj.slowDown = 1; % Run this in real time.
            obj.rate     = 0.04;   % with 25 fps
            
            % Copy the parameter vector:
            % Initialize the graphics
            obj.fig = figure();
            clf(obj.fig);
            set(obj.fig, 'Renderer','OpenGL');
            % Set some window properties
            set(obj.fig,'Name','2D-Output of a SLIP model');  % Window title
            set(obj.fig,'Color','w');         % Background color
%             set(obj.fig, 'position', get(0,'ScreenSize'));
%             box on
%             axis off;
            
            set(obj.fig, 'position', [200 200 750 600]);
            axis off;
            ax = gca;
            outerpos = ax.OuterPosition;
            ti = ax.TightInset; 
            left = outerpos(1) + ti(1);
            bottom = outerpos(2) + ti(2);
            ax_width = outerpos(3) - ti(1) - ti(3);
            ax_height = outerpos(4) - ti(2) - ti(4);
            ax.Position = [left bottom ax_width ax_height];
            box off;
            axis equal;
            
            % Define some arbitrary states:
            x = 1;
            y = 1.2;
            l_leg = 1;
            phi_body =0;

            % The representation of the back left leg as a line object:
            obj.SpringLine_l = DrawLegsLeftPointFeet(x,y,l_leg,phi_body);
            
            vert_x_out = [-20 100 100 -20];
            vert_y_out = [0 0 -40 -40];
            patch(vert_x_out, vert_y_out,'white');  
            
            % Draw the ground. It reaches from -2.5 to +6.5.
            h   = 0.01; % Height of the bar at the top
            n   = 5000;  % Number of diagonal stripes in the shaded area
            s   = 0.05; % Spacing of the stripes
            w   = 0.01; % Width of the stripes
            ext = 0.1;  % Length of the stripes
    
            % Create vertices by shifting a predefined pattern 'n' times to the right:
            v = [     -50,0;
                repmat([     0,    -h],n,1) + [-50+linspace(0,s*n,n)',zeros(n,1)];
                repmat([  -ext,-ext-h],n,1) + [-50+linspace(0,s*n,n)',zeros(n,1)];
                repmat([-ext+w,-ext-h],n,1) + [-50+linspace(0,s*n,n)',zeros(n,1)];
                repmat([     w,    -h],n,1) + [-50+linspace(0,s*n,n)',zeros(n,1)];
                -50+s*n+w,0];
            % Connect to faces:
            f = [1,2,4*n+1,4*n+2;
                 repmat([0,n,2*n,3*n],n,1) + repmat((1:n)',1,4)+1];

            % Color is uniformly black
            patch('faces', f, 'vertices', v);
            
            obj.Body = DrawBody(x,y);
            % The representation of the back right leg as a line object:
            obj.SpringLine_r = DrawLegsPointFeet(x,y,l_leg,phi_body);
         
            phi = linspace(0, pi/2, 10);
            vert_x = [0,sin(phi)*0.1,0];
            vert_x = [vert_x;vert_x;-vert_x;-vert_x]' + x;
            vert_y = [0,cos(phi)*0.1,0];
            vert_y = [vert_y;-vert_y;-vert_y;vert_y]' + y;
            
            obj.COGPatch = patch(vert_x, vert_y, cat(3,[1 0 1 0], [1 0 1 0],[1 0 1 0]),'LineWidth',3); 
            % Set up view:

            
        end
        
        
        % Updated function.  Is called by the integrator:
        function obj = update(obj,Y,tEvents,T)
        
        x        = Y(1);
        y        = Y(3);
        alphaL   = Y(5);
        alphaR   = Y(7);

        
        la       = 0.5; % half of the body length
        l_l      = 1;
        l_r      = 1;
        tL_TD = tEvents(1);
        tL_LO = tEvents(2);
        tR_TD = tEvents(3);
        tR_LO = tEvents(4);

        if ((T>tL_TD && T<tL_LO && tL_TD<tL_LO) || ((T<tL_LO || T>tL_TD) && tL_TD>tL_LO))
            contactL = true;
        else
            contactL = false;
        end
        if ((T>tR_TD && T<tR_LO && tR_TD<tR_LO) || ((T<tR_LO || T>tR_TD) && tR_TD>tR_LO))
            contactR = true;
        else
            contactR = false;
        end


        if contactL == true
            l_leg_L = y/cos(alphaL);    
        else    
            l_leg_L = l_l;
        end

        if contactR == true
            l_leg_R = y/cos(alphaR);    
        else    
            l_leg_R = l_r;
        end


        %  left Leg
        SetDrawLegsPointFeet(x,y,l_leg_L,alphaL,obj.SpringLine_l);    
        % The representation of the body as ellipse object:
        SetDrawBody(x,y, 0 ,obj.Body);
        %  right Leg
        SetDrawLegsPointFeet(x,y,l_leg_R,alphaR,obj.SpringLine_r);
        
        phi = linspace(0, pi/2, 10);
        vert_x = [0,sin(phi)*0.1,0];
        vert_x = [vert_x;vert_x;-vert_x;-vert_x]' + x;
        vert_y = [0,cos(phi)*0.1,0];
        vert_y = [vert_y;-vert_y;-vert_y;vert_y]' + y;
        
        set(obj.COGPatch,'XData', vert_x, 'YData', vert_y);

        axis([x-1.5,x+1.5,-0.3,2])
        box on;
        drawnow();
        end
        
    end
end