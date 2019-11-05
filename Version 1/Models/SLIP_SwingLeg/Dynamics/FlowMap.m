% *************************************************************************
%
% function dydt = FlowMap(y, z, p)
% 
% This MATLAB function defines the continuous dynamics of a simple SLIP
% (Spring Loaded Inverted Pendulum) model in 2D. The models current
% continuous and discrete states, as well as the model parameters are given
% by the calling routine and the derivative of the continuous states is
% returned.
%
% Input:  - A vector of continuous states 'y' 
%         - A vector of discrete states 'z' 
%         - A vector of model system parameters 'p'
%
% Output: - The derivative of the continuous state vector 'dydt'
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
%   See also HYBRIDDYNAMICS, JUMPMAP, JUMPSET, COMPUTEDIFFERENTIABLEFORCES 
%            CONTSTATEDEFINITION, DISCSTATEDEFINITION, SYSTPARAMDEFINITION,
%            EXCTSTATEDEFINITION, EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC, 
%            SYMBOLICCOMPUTATIONOFEQM. 
%
function dydt = FlowMap(t, y, z, p, SMA_L, SMA_R, varargin)
    global counter
    global SMA_L_database
    global SMA_R_database
    global store_flag
	% Get a mapping for the state and parameter vectors.  This allows us
    % to use a more readable syntax: "y(contStateIndices.dy)" instead of
    % "y(3)" while still operating with vectors and not with structs.
    % We keep the index-structs in memory to speed up processing
    persistent contStateIndices  systParamIndices discStateIndices
    if isempty(contStateIndices)  || isempty(systParamIndices) || isempty(discStateIndices)
        [~, ~, contStateIndices] = ContStateDefinition();
        [~, ~, systParamIndices] = SystParamDefinition();
        [~, ~, discStateIndices] = DiscStateDefinition();
    end
    
    % Copying the state-vector to the derivatives ensures a correct vector
    % size
    dydt = y;

    % Define all the parameters here:
    M      =   p(systParamIndices.m_0);
    g      =   p(systParamIndices.g);
    kvm    =   p(systParamIndices.kh);  % value of k over m
    k      =   p(systParamIndices.k);
    kvj    =   0; 
    
    alpha  =   y(contStateIndices.alpha);
    phiL   =   y(contStateIndices.phiL);
    phiR   =   y(contStateIndices.phiR);
    
    l_legL = p(systParamIndices.l_0);
    r_legL = p(systParamIndices.l_0);
    
    switch z(discStateIndices.lphase)
        case 1 %(flight = 1)
            LegL = l_legL;
            Fl=0;
            phi_L = phiL; % in body frame
        case 2 %(stance = 2)
            LegL = sqrt((y(contStateIndices.x)-z(discStateIndices.lcontPt))^2 + (y(contStateIndices.y)-0)^2);
            if isnan(k)
                u = l_legL-LegL;
                counter = counter + 1;
                SMA_L.eps = u*3*SMA_L.r/(8*3.1415*SMA_L.N*SMA_L.R^2);
                SMA_L.T = SMA_L.T_function(t);
                [SMA_L] = OneD_SMA_Model(counter, SMA_L);
                F_sma = SMA_L.sigma*2*3.1415*SMA_L.r^3/(3*SMA_L.R)/SMA_L.norm;
                % fprintf('Left: %d, %d, %d\n', t, SMA_L.eps, SMA_L.area*SMA_L.sigma/SMA_L.norm/u)
                Fl=F_sma;
            else
                Fl=k*(l_legL-LegL);
            end
            phi_L = atan2(z(discStateIndices.lcontPt)-y(contStateIndices.x), y(contStateIndices.y)-0) - alpha; % in body frame

    end
            
    switch z(discStateIndices.rphase)
        case 1 %(flight = 1)
            LegR = r_legL;
            Fr=0;
            phi_R = phiR; % convert to inertia frame
        case 2 %(stance = 2)
            LegR = sqrt((y(contStateIndices.x)-z(discStateIndices.rcontPt))^2 + (y(contStateIndices.y)-0)^2);
            if isnan(k)
                u = r_legL-LegR;
                counter = counter + 1;
                SMA_R.eps = u*3*SMA_R.r/(8*3.1415*SMA_R.N*SMA_R.R^2);
                SMA_R.T = SMA_R.T_function(t);
                [SMA_R] = OneD_SMA_Model(counter, SMA_R);
                F_sma = SMA_R.sigma*2*3.1415*SMA_R.r^3/(3*SMA_R.R)/SMA_R.norm;
%                 try
%                     fprintf('Right: %d, %d, %d, %d, %d\n', t, SMA_R.eps, 20*(r_legL-LegR), F_sma, SMA_R_database.index)
%                 catch
%                     fprintf('Right: %d, %d, %d, %d, 1\n', t, SMA_R.eps, 20*(r_legL-LegR), F_sma)
%                 end
                Fr=F_sma;
            else
                Fr=k*(r_legL-LegR);
            end
            phi_R = atan2(z(discStateIndices.rcontPt)-y(contStateIndices.x), y(contStateIndices.y)-0) - alpha; % in body frame
    end                         
    
    if store_flag && isnan(k)
        store(SMA_L, SMA_R)
%         scatter(t,Fr,'k')
    end
    Fx=-Fl*sin(phi_L+alpha)-Fr*sin(phi_R+alpha)-SMA_R.F_external;
    Fy= Fl*cos(phi_L+alpha)+Fr*cos(phi_R+alpha);
    
    % Compute the derivative of all states according to the phase:
    dydt(contStateIndices.x)      = y(contStateIndices.dx);
    dydt(contStateIndices.y)      = y(contStateIndices.dy);
    dydt(contStateIndices.alpha)  = y(contStateIndices.dalpha);
    
    dydt(contStateIndices.dx) = Fx/M;
    dydt(contStateIndices.dy) = (Fy - M*g)/M;
    dydt(contStateIndices.dalpha) = kvj*(phi_L+phi_R);   
    
    if (z(discStateIndices.lphase)==2) % left leg in stance
        dydt(contStateIndices.phiL)  = (-y(contStateIndices.dx)*cos(phi_L + alpha) ...
                                        -y(contStateIndices.dy)*sin(phi_L + alpha))/LegL - y(contStateIndices.dalpha);
        dydt(contStateIndices.dphiL) = (-dydt(contStateIndices.dx)*cos(phi_L + alpha) ... 
                                        -dydt(contStateIndices.dy)*sin(phi_L + alpha))/LegL +...
                                        2*(y(contStateIndices.dphiL)+y(contStateIndices.dalpha))...
                                         *(+y(contStateIndices.dx)*sin(phi_L+ alpha) ...
                                           -y(contStateIndices.dy)*cos(phi_L+ alpha))/LegL;  
    else  % left leg in the air
        dydt(contStateIndices.phiL) = y(contStateIndices.dphiL);
        dydt(contStateIndices.dphiL) =  -(M*phi_L*kvm + Fy*LegL*sin(phi_L + alpha) + Fx*LegL*cos(phi_L + alpha))/(M*LegL^2) ...
                                        - kvj*(phi_L+phi_R);

    end
    
    if  (z(discStateIndices.rphase)==2) % when right leg is in stance
        dydt(contStateIndices.phiR)  = (-y(contStateIndices.dx)*cos(phi_R + alpha ) ...
                                        -y(contStateIndices.dy)*sin(phi_R + alpha ))/LegR - y(contStateIndices.dalpha); 
        dydt(contStateIndices.dphiR) = (-dydt(contStateIndices.dx)*cos(phi_R+ alpha) ...
                                        -dydt(contStateIndices.dy)*sin(phi_R + alpha))/LegR +...
                                        2*(y(contStateIndices.dphiR)+y(contStateIndices.dalpha))...
                                         *(y(contStateIndices.dx)*sin(phi_R+ alpha)...
                                          -y(contStateIndices.dy)*cos(phi_R+ alpha))/LegR;    
    else % when right leg is in swing
        dydt(contStateIndices.phiR)  = y(contStateIndices.dphiR); 
        dydt(contStateIndices.dphiR) =  -(M*phi_R*kvm + Fy*LegR*sin(phi_R + alpha) + Fx*LegR*cos(phi_R + alpha))/(M*LegR^2) ...
                                        - kvj*(phi_L+phi_R);
  
    end

    dydt(contStateIndices.t)=1;
    %
    % ************************************
    % ************************************
end
% *************************************************************************
% *************************************************************************
    