function residual = f_Continuation(X,yLast)

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
    % Event timing:
    tL_TD    = X(8);
    tL_LO    = X(9);
    tR_TD    = X(10);
    tR_LO    = X(11);
    
    % Move all timing values into [0..tR_TD]
    while tL_TD < 0
         tL_TD = tL_TD + tR_TD;
    end
    while tL_TD > tR_TD
         tL_TD = tL_TD - tR_TD;
    end 
    while tL_LO < 0
         tL_LO = tL_LO + tR_TD;
    end
    while tL_LO > tR_TD
         tL_LO = tL_LO - tR_TD;
    end 
    while tR_LO < 0
         tR_LO = tR_LO + tR_TD;
    end
    while tR_LO > tR_TD
         tR_LO = tR_LO - tR_TD;
    end
    
    %**********************************************************************
    % Integration
    %**********************************************************************
    % Set up start of integration:
    T_START = 0;
    Y_START = [x0; dx0; y0; dy0; alphaL0; dalphaL0; alphaR0; dalphaR0];
    % Integrate motion in 4 steps, which are determined by the order of the
    % event times: 
    % Determine this order (iEVENT(i) is the Eventnumber of the ith event)
    [tEVENT,iEVENT] = sort([tL_TD,tL_LO,tR_TD,tR_LO]);
    % Prepare output:
    T = [];
    Y = [];
    Y_EVENT = zeros(4,8);
    for i = 1:4 %Integrate motion i/4
        % Figure out the current contact configuration (this is used in the
        % dynamics function)
        t_ = T_START+eps;
        if ((t_>tL_TD && t_<tL_LO && tL_TD<tL_LO) || ((t_<tL_LO || t_>tL_TD) && tL_TD>tL_LO))
            contactL = true;
        else
            contactL = false;
        end
        if ((t_>tR_TD && t_<tR_LO && tR_TD<tR_LO) || ((t_<tR_LO || t_>tR_TD) && tR_TD>tR_LO))
            contactR = true;
        else
            contactR = false;
        end
        % Set up solver
        %************************
        % EITHER: Variable time step solver:
        % Setup ode solver options:
        %options = odeset('RelTol',1e-6,'AbsTol',1e-6);
        %[T_PART,Y_PART] = ode45(@ode,[T_START,tEVENT(i)],Y_START,options);
        %************************
        % OR: Fixed time step solver:
        dt = 0.05;
        T_PART = [T_START:dt:tEVENT(i),tEVENT(i)]';
        Y_PART = ode5(@ode,T_PART,Y_START);
        % Event handlers:
        if iEVENT(i)==1
            % If this is EVENT 1, append touchdown L:
            T_PART=[T_PART;T_PART(end)];
            Y_PART=[Y_PART;Y_PART(end,:)];
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
     
    % Show Trajectory (Can be removed):
%     P = [tL_TD,tL_LO,tR_TD,tR_LO,0,0,k,omega];
%     ShowTrajectory(T,Y,P);
    
    %**********************************************************************
    % Compute Residuals
    %**********************************************************************
    % Relabel event values:
    YL_TD = Y_EVENT(1,:)';
    YL_LO = Y_EVENT(2,:)';
    YR_TD = Y_EVENT(3,:)';
    YR_LO = Y_EVENT(4,:)';
    % Compute residuals
    % Periodicity:
    residual = zeros(12,1);
    residual(1:7) = Y(1,2:8).'-YR_TD(2:8);
    % At the touch-down events, the feet have to be on the ground:
    residual(8)   = YL_TD(3) -  cos(YL_TD(5));
    residual(9)   = YR_TD(3) -  cos(YR_TD(7));
    % At the lift-off events, the feet also have to be on the ground:
    residual(10)  = YL_LO(3) -  cos(YL_LO(5));
    residual(11)  = YR_LO(3) -  cos(YR_LO(7));
    residual(12)  = norm(yLast(2:end) - X(1:7)) - stepSize;
    
    

    %**********************************************************************
    % Dynamics Function
    %**********************************************************************
    function dydt_ = ode(t_,y_)
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
        if contactL
            Fx = Fx - (1- y/cos(alphaL))*k*sin(alphaL);
            Fy = Fy + (1- y/cos(alphaL))*k*cos(alphaL);
        end
        if contactR
            Fx = Fx - (1- y/cos(alphaR))*k*sin(alphaR);
            Fy = Fy + (1- y/cos(alphaR))*k*cos(alphaR);
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