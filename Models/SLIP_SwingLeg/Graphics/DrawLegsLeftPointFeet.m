
function  LegParts=DrawLegsLeftPointFeet(x_o,y_o,l_leg, gamma_leg, MVF)
comp= l_leg - 1;
T= [ cos(gamma_leg), -sin(gamma_leg);
         sin(gamma_leg),  cos(gamma_leg)];
% Spring Part 1***********************************
vert_xsp1([1 3 5 7  9 11 13 15])  = - 0.09;
vert_xsp1([2 4 6 8 10 12 14])      =+ 0.09;     
vert_ysp1 = [-0.4,linspace(-0.4,-0.8-comp,13),-0.8-comp]; % Compress
           
vert_sp1_rot = T*[vert_xsp1;vert_ysp1];% Rotate
vert_xsp1= vert_sp1_rot (1,:) +  x_o ;
vert_ysp1 = vert_sp1_rot (2,:) + y_o ;

XCOl = 202;          
COLOR = [XCOl XCOl XCOl]/256;           
vsp1=[vert_xsp1',vert_ysp1'];
fsp1=[linspace(1,14,14)',linspace(2,15,14)'];
Red = [0.6350, 0.0780, 0.1840];
Blue = [0 68/256 .5]; %[0 68 158]/256
MVF_max = 1;
if isnan(MVF) || isempty(MVF)
    spring_color=Red;
else
    spring_color = Blue*MVF/MVF_max+ Red*(MVF_max-MVF)/MVF_max;
end
% spring_color = [0 68 158]/256;
L1=patch('faces', fsp1, 'vertices', vsp1,'linewidth',5,'EdgeColor',spring_color);

% Upper Leg**************************************
% 1. outline of upper leg
xul(1:3)=[-0.12,-0.12,-0.05];
yul(1:3)=[-0.4,-0.35,-0.35];
phi = linspace(pi,0,20);
xul(4:23) =  cos(phi)*0.05;
yul(4:23) =  sin(phi)*0.05;
xul(24:26) =  [0.05,0.12,0.12];
yul(24:26) =  fliplr(yul(1:3));

xyul_rot = T*[xul;yul];% Rotate
xul= xyul_rot (1,:) +  x_o ;
yul =xyul_rot (2,:) +  y_o ;

vul=[xul',yul'];
ful=[26,1:1:25];

% 2. shade the upper leg
% Create graphic objects:
% The representation of the COG as patch object:
phi = linspace(0, pi/2, 10);
vert_x = [0,sin(phi)*0.02,0];
vert_x = [vert_x;vert_x;-vert_x;-vert_x]'+x_o  ;
vert_y = [0,cos(phi)*0.02,0];
vert_y = [vert_y;-vert_y;-vert_y;vert_y]' +  y_o - comp-1;

% Lower Leg ***************************************************************
xll(1:2)   = [-0.03,-0.03];
yll(1:2)   = [-0.70,-0.81];
phi        = linspace(pi/2,0,20);
xll(3:22)  = cos(phi)*0.10 - 0.1;
yll(3:22)  = sin(phi)*0.19 - 1;
phi        = linspace(pi/2,pi/2+2*pi,20);
xll(23:42) = cos(phi)*0.01;
yll(23:42) = sin(phi)*0.01 - 1;
phi        = linspace(pi,pi/2,10);
xll(43:52) = cos(phi)*0.10 + 0.1;
yll(43:52) = sin(phi)*0.19 - 1;
xll(53:54) = [+0.03,+0.03];
yll(53:54) = [-0.81,-0.70];
vecS = [x_o;y_o];

vll  = LTrans([xll;yll - comp],gamma_leg,vecS);
fll  = 1:54; % select point
fsp2 = [1:2:13; 2:2:14]';

L4=patch('faces', fll, 'vertices', vll,'linewidth',3,'FaceColor',COLOR,'EdgeColor',[0 0 0]);

% Spring Part 2 **********************************
vert_xsp2([1 3 5 7  9 11 13 15])  = - 0.09;
vert_xsp2([2 4 6 8 10 12 14])     = + 0.09;     
vert_ysp2= [-0.4,linspace(-0.4,-0.8-comp,13),-0.8-comp];% Compressed
 
vert_sp2_rot = T*[vert_xsp2;vert_ysp2];% Rotated
vert_xsp2= vert_sp2_rot (1,:) +  x_o ;
vert_ysp2 = vert_sp2_rot (2,:) + y_o ;

vsp2=[vert_xsp2(1:14)',vert_ysp2(1:14)'];
fsp2=[1,2;3,4;5,6;7,8; 9,10;11,12;13,14];
L5=patch('faces', fsp2, 'vertices', vsp2,'linewidth',5,'EdgeColor',spring_color);

L2=patch('faces', ful, 'vertices', vul,'linewidth',3,'FaceColor',COLOR);
LegParts = struct('L_Sp1',L1,'L_Upo',L2,'L_low',L4,'L_Sp2',L5);

function VecTrans = LTrans(VecTrans, gamma, vecS)
    % Get absolute angle of each object and the vector of translational
    % motion. VecTrans is a 2 by n vector and the output VecTrans is a
    % n by 2 vector
    % (which must be on the MATLAB search path):
    % Rotational matrix:
    RotM = [ cos(gamma), -sin(gamma);
             sin(gamma),  cos(gamma)];
    vec_rot = RotM*VecTrans;    % Rotate 
    vec_ = vec_rot + vecS*ones(1,size(VecTrans,2)); % Shift
    VecTrans = vec_'; % Change to column vector.
end

end
