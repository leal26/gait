%% (a) Display a stored solution
path(pathdef)
CurrentPath = pwd;
addpath(CurrentPath)
addpath([CurrentPath,filesep,'BipedalGaits;'])
addpath([CurrentPath,filesep,'Graphics;'])
addpath([CurrentPath,filesep,'StoredFunctions;'])
load('R2_v1_ALL.mat');
close all; clc;
k = 150;
xCYC = config(:,k);

% Show animation
[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex(xCYC);
ShowTrajectory(T,Y,P,'Test')

[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex(xCYC);
figure(2)
plot(T,Y,'LineWidth',2)
grid on; box on
xlim([0 T(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');

leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
    ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);

%% Phase diagram
figure(2)
plot(Y(:,3), Y(:,4), 'LineWidth',2)
grid on; box on
xlim([0 T(end)]);
xlabel('$y$');
ylabel('$\dot{y}$')

%% (b) Display a whole branch
load('R2_v1_ALL.mat');
figure(3)
plot3(config(1,:),config(6,:),config(2,:),'LineWidth',2);
view([-45 45]); axis square;
box on; grid on;
xlabel('$\dot{x}  [\sqrt{gl_o}]$','Interpreter','LaTex')
ylabel('$\alpha_R$   $[rad]$','Interpreter','LaTex')
zlabel('$y  [l_o]$','Interpreter','LaTex')


