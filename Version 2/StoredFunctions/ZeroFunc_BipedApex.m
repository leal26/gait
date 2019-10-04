function [T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex(X, SMA_L, SMA_R)
    global counter
    %**********************************************************************
    % General Preparation
    %**********************************************************************
    % Define model parameters:
    k = 20;
    omega = sqrt(5);
 
    % Extract from X-vector:
    % Initial states
    x0       = 0;
    dx0      = X(1); 
    y0       = X(2);
    dy0      = X(3);
    alphaL0  = X(4);
    dalphaL0 = X(5);
    alphaR0  = X(6);
    dalphaR0 = X(7);
    
    %**********************************************************************
    % Integration
    %**********************************************************************
    % Set up start of integration:
    T_START = 0;
    Y_START = [x0, dx0, y0, dy0, alphaL0, dalphaL0, alphaR0, dalphaR0];
    % Integrate motion in 4 steps, which are determined by the order of the
    % event times: 
    % Determine this order (iEVENT(i) is the Eventnumber of the ith event)
    [tEVENT,iEVENT] = sort([tL_TD,tL_LO,tR_TD,tR_LO,tAPEX]);
    % Prepare output:
    T = [];
    Y = [];
    Y_EVENT = zeros(4,8);

    for i = 1:5 %Integrate motion i/4
        % Set up solver
  
        %************************
        % EITHER: Variable time step solver:
        % Setup ode solver options: 'NormControl','on' 'OutputFcn',@odeplot
        % 'Stats','on'
            
        options = odeset('RelTol',1e-12,'AbsTol',1e-12);
        
        if abs(T_START - tEVENT(i))<1e-12 
            Y_PART = Y_START;
            T_PART = T_START;
        else   
            counter = 1;
            [T_PART,Y_PART, out] = ode23s(@(t,y) ode(t,y,SMA_L, SMA_R),[T_START,tEVENT(i)],Y_START,options);
        end   
        %************************
       
        % Event handlers:
        if iEVENT(i)==1
            % By default, we assume are row vectors
            % If this is EVENT 1, append touchdown L:
            T_PART=[T_PART;T_PART(end)]; % This will not do anything when T_PART is a scalar
            Y_PART=[Y_PART;Y_PART(end,:)];% When Y_PART is a vector, Y_PART(end,:) == 0, this will cause an error in the dimension
            % Y_PART(end,2) will through an error that 'Index exceeds matrix dimensions.'
            Y_PART(end,6) = -(Y_PART(end,2)+Y_PART(end,4)*tan(Y_PART(end,5)))/(Y_PART(end,3)*(tan(Y_PART(end,5))^2+1));
        end
        if iEVENT(i)==3
            % If this is EVENT 3, append touchdown R:
            T_PART=[T_PART;T_PART(end)];
            Y_PART=[Y_PART;Y_PART(end,:)];
            Y_PART(end,8) = -(Y_PART(end,2)+Y_PART(end,4)*tan(Y_PART(end,7)))/(Y_PART(end,3)*(tan(Y_PART(end,7))^2+1));
        end
        % Compose total solution
        T = [T;T_PART];
        Y = [Y;Y_PART];

        % Extract values at Events
        Y_EVENT(iEVENT(i),:)=Y(end,:);
        % Prepare initial values for next integration:
        T_START = T(end);
        Y_START = Y(end,:);
    end

  
  
    %**********************************************************************
    % Output variables
    %**********************************************************************
    % Relabel event values:
    YL_TD = Y_EVENT(1,:)';
    YL_LO = Y_EVENT(2,:)';
    YR_TD = Y_EVENT(3,:)';
    YR_LO = Y_EVENT(4,:)';
    YAPEX = Y_EVENT(5,:)';
    
    if nargout > 1
       P  = [tL_TD,tL_LO,tR_TD,tR_LO,tAPEX,k,omega]; 
    end
    
    %**********************************************************************
    % Dynamics Function
    %**********************************************************************
    function dydt_ = ode(t,y_, SMA_L, SMA_R)
        % Extract individual states:
        x        = y_(1);
        dx       = y_(2);
        y        = y_(3);
        dy       = y_(4);
        alphaL   = y_(5);
        dalphaL  = y_(6);
        alphaR   = y_(7);
        dalphaR  = y_(8);
        
        % Compute forces acting on the main body (only legs in contact
        % contribute): 
        Fx = 0;
        Fy = 0;
%         if SMA.counter == 1
%             SMA.eps_0 = 
        if contactL
            % non-dimensional displacement equal strain
            counter = counter + 1;
            u = (1-y/cos(alphaL));
            SMA_L.eps = u*20/1.138989e+04;
            SMA_L.T = SMA_L.T_function(t);
            [SMA_L] = OneD_SMA_Model(SMA_L);
            F_sma = SMA_L.area*SMA_L.sigma/SMA_L.norm; % k*SMA_L.eps;
            fprintf('Left: %d, %d, %d, %d\n', t, SMA_L.eps, SMA_L.area*SMA_L.sigma/SMA_L.norm/u, k)
            Fx = Fx - F_sma*sin(alphaL);
            Fy = Fy + F_sma*cos(alphaL);
            store(SMA_L, 0)
            scatter(t, F_sma,[],'b')
            drawnow() 
        end
        if contactR
            % non-dimensional displacement equal strain
            u = (1-y/cos(alphaR));
            counter = counter + 1;
            SMA_R.eps = u*20/1.138989e+04;
            SMA_R.T = SMA_R.T_function(t);
            [SMA_R] = OneD_SMA_Model(SMA_R);
            F_sma = SMA_R.area*SMA_R.sigma/SMA_R.norm;% F_sma = k*SMA_R.eps/10;  
            fprintf('Right: %d, %d, %d, %d\n', t, SMA_R.eps, SMA_R.area*SMA_R.sigma/SMA_R.norm/u, k)
            Fx = Fx - F_sma*sin(alphaR);
            Fy = Fy + F_sma*cos(alphaR);
            store(0, SMA_R)
            scatter(t, F_sma,[],'r')
            drawnow() 
        end

        
        % Compute main body acceleration:
        ddx = Fx;
        ddy = Fy-1;

        % Compute leg acceleration:
        % If leg is in swing, apply dynamics, if leg is in contact, apply
        % kinematic constraint on position derivative and run a controller
        % to compute velocity derivative, such that the velocity state
        % matches the constrained position derivative.
        % The contact configuration is set outside, since it is constant
        % for each integration period.
        if contactL % If left leg is in stance
            ddalphaL = - 2*tan(alphaL)*dalphaL^2 - (2*dy*dalphaL)/y - (ddx + ddy*tan(alphaL))/(y*(tan(alphaL)^2 + 1));
        else % If left leg is in air
            ddalphaL = -cos(alphaL)*Fx-sin(alphaL)*Fy-alphaL*omega^2;
        end
        if contactR % If right leg is in stance
            ddalphaR = - 2*tan(alphaR)*dalphaR^2 - (2*dy*dalphaR)/y - (ddx + ddy*tan(alphaR))/(y*(tan(alphaR)^2 + 1));
        else % If right leg is in air
            ddalphaR = -cos(alphaR)*Fx-sin(alphaR)*Fy-alphaR*omega^2;
        end
        dydt_ = [dx;ddx;dy;ddy;dalphaL;ddalphaL;dalphaR;ddalphaR];
    end
end
