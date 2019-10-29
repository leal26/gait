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
classdef SLIP_Model_Graphics_AdvancedPointFeet < OutputCLASS 
    % Private attributes:
    properties 
        % The output window
        fig; 
        % The parameter vector:
        p;
        % Patch and line objects used in the graphical representation
        COGPatch;
        Body;
        
        SpringLine_l;
        SpringLine_r;

        video;
    end
    % Public methods:
    methods
        % Constructor:
        function obj = SLIP_Model_Graphics_AdvancedPointFeet(p)
            global SMA_L_database
            global SMA_R_database
            obj.slowDown = 1; % Run this in real time.
            obj.rate     = 0.01;   % with 25 fps
            
            % Copy the parameter vector:
            obj.p = p;
            
            % Initialize the graphics
            obj.fig = figure();
            clf(obj.fig);
            set(obj.fig, 'Renderer','OpenGL');
            % Set some window properties
            set(obj.fig,'Name','2D-Output of a SLIP model');  % Window title
            set(obj.fig,'Color','w');         % Background color
            set(obj.fig, 'position', [100 100 600 600]);
            ax = gca;
            outerpos = ax.OuterPosition;
            ti = ax.TightInset; 
            left = outerpos(1) + ti(1);
            bottom = outerpos(2) + ti(2);
            ax_width = outerpos(3) - ti(1) - ti(3);
            ax_height = outerpos(4) - ti(2) - ti(4);
            ax.Position = [left bottom ax_width ax_height];
            
            box on
            axis off;
            % Define some arbitrary states:
            x = 1;
            y = 1.2;
            l_leg = 1;
            phi_body =0;
            try
                MVF = SMA_L_database.MVF(SMA_L_database.index,1);
            catch
                MVF = 0;
            end
            % The representation of the back left leg as a line object:
            obj.SpringLine_l = DrawLegsLeftPointFeet(x,y,l_leg,phi_body,MVF);
            
            vert_x_out = [-20 100 100 -20];
            vert_y_out = [0 0 -40 -40];
            patch(vert_x_out, vert_y_out,'white');  
            
            % Draw the ground. It reaches from -0.5 to +8.5.
            h   = 0.01; % Height of the bar at the top
            n   = 180*10;  % Number of diagonal stripes in the shaded area
            s   = 0.1;  % Spacing of the stripes
            w   = 0.01; % Width of the stripes
            ext = 0.1;  % Length of the stripes
            
            sp = -15;

            v = [sp,0;
                 repmat([0,-h],n,1) + [sp+linspace(0,s*n,n)',zeros(n,1)];
                 repmat([-ext,-ext-h],n,1) + [sp+linspace(0,s*n,n)',zeros(n,1)];
                 repmat([-ext+w,-ext-h],n,1) + [sp+linspace(0,s*n,n)',zeros(n,1)];
                 repmat([w,0-h],n,1) + [sp+linspace(0,s*n,n)',zeros(n,1)];
                 sp+s*n+w,0];
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

            obj.video = VideoWriter('test','MPEG-4');
            obj.video.Quality = 100;
            open(obj.video);
        end
        
        
        % Updated function.  Is called by the integrator:
        function obj = update(obj, y, z, ~, ~)
        global SMA_L_database
        global SMA_R_database   
        % Get a mapping for the state and parameter vectors.  This allows us
        % to use a more readable syntax: "y(contStateIndices.dy)" instead of
        % "y(3)" while still operating with vectors and not with structs.
        % We keep the index-structs in memory to speed up processing
        persistent contStateIndices  systParamIndices discStateIndices
        if isempty(contStateIndices) || isempty(discStateIndices) || isempty(systParamIndices) 
            [~, ~, contStateIndices] = ContStateDefinition();
            [~, ~, discStateIndices] = DiscStateDefinition();
            [~, ~, systParamIndices] = SystParamDefinition();
        end
            
        phi_body = y(contStateIndices.alpha);
        x_ = y(contStateIndices.x);
        y_ = y(contStateIndices.y);

 
       switch z(discStateIndices.lphase)
        case 1 %(flight = 1)
            LegL = 1;
            phi_L=y(contStateIndices.phiL)+phi_body; % in abs frame
        case 2 %(stance = 2)
            LegL = sqrt((y(contStateIndices.x)-z(discStateIndices.lcontPt))^2 + (y(contStateIndices.y)-0)^2) ;
            phi_L = atan2(z(discStateIndices.lcontPt)-y(contStateIndices.x), y(contStateIndices.y)-0);
        end

        switch z(discStateIndices.rphase)
            case 1 %(flight = 1)
                LegR = 1;
                phi_R=y(contStateIndices.phiR)+phi_body;% in abs frame
            case 2 %(stance = 2)
                LegR = sqrt((y(contStateIndices.x)-z(discStateIndices.rcontPt))^2 + (y(contStateIndices.y)-0)^2);
                phi_R = atan2(z(discStateIndices.rcontPt)-y(contStateIndices.x), y(contStateIndices.y)-0);
        end    
   
        try
            MVF_L = SMA_L_database.MVF(SMA_L_database.index,1);
        catch
            MVF_L = 0;
        end
        try
            MVF_R = SMA_R_database.MVF(SMA_R_database.index,1);
        catch
            MVF_R = 0;
        end
        %  left Leg
        SetDrawLegsPointFeet(x_,y_,LegL,phi_L,obj.SpringLine_l, MVF_L);    
        % The representation of the body as ellipse object:
        SetDrawBody(x_,y_, phi_body ,obj.Body);
        %  right Leg
        SetDrawLegsPointFeet(x_,y_,LegR,phi_R,obj.SpringLine_r, MVF_R);
        
        phi = linspace(0, pi/2, 10);
        vert_x = [0,sin(phi)*0.1,0];
        vert_x = [vert_x;vert_x;-vert_x;-vert_x]' + x_;
        vert_y = [0,cos(phi)*0.1,0];
        vert_y = [vert_y;-vert_y;-vert_y;vert_y]' + y_;
        
        set(obj.COGPatch,'XData', vert_x, 'YData', vert_y);

        axis([x_-1,x_+1,-0.2,1.8])
        box on;
        drawnow();
        writeVideo(obj.video,getframe(gcf));
        end
        
        
        % Updated function.  Is called by the integrator:
        function obj = update2(obj, y, z, ~, ~)
           
        % Get a mapping for the state and parameter vectors.  This allows us
        % to use a more readable syntax: "y(contStateIndices.dy)" instead of
        % "y(3)" while still operating with vectors and not with structs.
        % We keep the index-structs in memory to speed up processing
        persistent contStateIndices  systParamIndices discStateIndices
        if isempty(contStateIndices) || isempty(discStateIndices) || isempty(systParamIndices) 
            [~, ~, contStateIndices] = ContStateDefinition();
            [~, ~, discStateIndices] = DiscStateDefinition();
            [~, ~, systParamIndices] = SystParamDefinition();
        end
            
        phi_body = y(contStateIndices.alpha);
        x_ = y(contStateIndices.x);
        y_ = y(contStateIndices.y);

 
       switch z(discStateIndices.lphase)
        case 1 %(flight = 1)
            LegL = 1;
            phi_L=y(contStateIndices.phiL)+phi_body; % in abs frame
        case 2 %(stance = 2)
            LegL = sqrt((y(contStateIndices.x)-z(discStateIndices.lcontPt))^2 + (y(contStateIndices.y)-0)^2) ;
            phi_L = atan2(z(discStateIndices.lcontPt)-y(contStateIndices.x), y(contStateIndices.y)-0);
        end

        switch z(discStateIndices.rphase)
            case 1 %(flight = 1)
                LegR = 1;
                phi_R=y(contStateIndices.phiR)+phi_body;% in abs frame
            case 2 %(stance = 2)
                LegR = sqrt((y(contStateIndices.x)-z(discStateIndices.rcontPt))^2 + (y(contStateIndices.y)-0)^2);
                phi_R = atan2(z(discStateIndices.rcontPt)-y(contStateIndices.x), y(contStateIndices.y)-0);
        end    
   
            
        % left Leg
        SetDrawLegsPointFeet(x_,y_,LegR,phi_R,obj.SpringLine_l);    
        % The representation of the body as ellipse object:
        SetDrawBody(x_,y_, phi_body ,obj.Body);
        % right Leg
        SetDrawLegsPointFeet(x_,y_,LegL,phi_L,obj.SpringLine_r);
        
        phi = linspace(0, pi/2, 10);
        vert_x = [0,sin(phi)*0.1,0];
        vert_x = [vert_x;vert_x;-vert_x;-vert_x]' + x_;
        vert_y = [0,cos(phi)*0.1,0];
        vert_y = [vert_y;-vert_y;-vert_y;vert_y]' + y_;
        
        set(obj.COGPatch,'XData', vert_x, 'YData', vert_y);
        xlim([x_-1,x_+1]);
        box on;
        drawnow();
        end
    end
end