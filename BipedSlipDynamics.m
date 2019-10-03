close all; clc; clear all;
global SMA_L_database
global SMA_R_database

path(pathdef)
CurrentPath = pwd;
addpath(CurrentPath)
addpath([CurrentPath,filesep,'inputs;'])
addpath([CurrentPath,filesep,'Graphics;'])
addpath([CurrentPath,filesep,'BipedalGaits;'])
addpath([CurrentPath,filesep,'StoredFunctions;'])
addpath([CurrentPath,filesep,'SMA_temperature_strain_driven;'])
%% (a) Define input parameters

IP.phase = 0.;
IP.duty = 1.;
IP.frequency = 1;
IP.amplitude = 324.6427+20;
IP.mass = 10; % kg (used for normalizing)
IP.gravity = 9.80665; % m/s2 (used for normalizing)
[SMA_L, SMA_R] = define_SMA(IP, IP);

%% (a) Display a stored solution
load('R2_v1_ALL.mat');
k = 150;
xCYC = config(:,k);

% Show animation
figure(50)
hold on;
[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex(xCYC, SMA_L, SMA_R);
ShowTrajectory(T,Y,P,'Test')
%%
figure(10)
subplot(3,2,1)
hold on
plot(T,SMA_L_database.eps(1:length(T)),'LineWidth',2)
plot(T,SMA_R_database.eps(1:length(T)),'LineWidth',2)
legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Strain','Interpreter','LaTex')

subplot(3,2,2)
hold on
plot(T,SMA_L_database.T(1:length(T)),'LineWidth',2)
plot(T,SMA_R_database.T(1:length(T)),'LineWidth',2)
legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Temperature (K)','Interpreter','LaTex')

subplot(3,2,3)
hold on
plot(T,SMA_L_database.sigma(1:length(T)),'LineWidth',2)
plot(T,SMA_R_database.sigma(1:length(T)),'LineWidth',2)
legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Stress (Pa)','Interpreter','LaTex')

subplot(3,2,4)
hold on
plot(T,SMA_L_database.MVF(1:length(T)),'LineWidth',2)
plot(T,SMA_R_database.MVF(1:length(T)),'LineWidth',2)
legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('$\xi$','Interpreter','LaTex')

subplot(3,2,5)
hold on
plot(T,SMA_L_database.eps_t(1:length(T)),'LineWidth',2)
plot(T,SMA_R_database.eps_t(1:length(T)),'LineWidth',2)
legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Transformation strain','Interpreter','LaTex')

subplot(3,2,6)
hold on
plot(T,SMA_L_database.E(1:length(T)),'LineWidth',2)
plot(T,SMA_R_database.E(1:length(T)),'LineWidth',2)
legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Young Modulus (Pa)','Interpreter','LaTex')

%%
% [residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex(xCYC, SMA_L, SMA_R);
figure(3)
plot(T,Y,'LineWidth',2)
grid on; box on
xlim([0 T(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');

leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
    ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);

%% Phase diagram
figure(4)
plot(Y(:,3), Y(:,4), 'LineWidth',2)
grid on; box on
xlim([0 T(end)]);
xlabel('$y$','Interpreter','LaTex');
ylabel('$\dot{y}$','Interpreter','LaTex')

%% (b) Display a whole branch
load('R2_v1_ALL.mat');
figure(5)
plot3(config(1,:),config(6,:),config(2,:),'LineWidth',2);
view([-45 45]); axis square;
box on; grid on;
xlabel('$\dot{x}  [\sqrt{gl_o}]$','Interpreter','LaTex')
ylabel('$\alpha_R$   $[rad]$','Interpreter','LaTex')
zlabel('$y  [l_o]$','Interpreter','LaTex')


