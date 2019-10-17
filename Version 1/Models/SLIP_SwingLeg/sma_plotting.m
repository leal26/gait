function [] = sma_plotting(T, P)
global SMA_L_database
global SMA_R_database

color_l = [127/256,127/256,127/256];
color_r = [0/256,45/256,98/256];

%%
figure
subplot(3,2,1)
hold on
plot(T,SMA_L_database.eps(1:length(T)),'LineWidth',2,'color',color_l)
plot(T,SMA_R_database.eps(1:length(T)),'LineWidth',2,'color',color_r)
legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Strain','Interpreter','LaTex')

subplot(3,2,2)
hold on
plot(T,SMA_L_database.T(1:length(T)),'LineWidth',2,'color',color_l)
plot(T,SMA_R_database.T(1:length(T)),'LineWidth',2,'color',color_r)
legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Temperature (K)','Interpreter','LaTex')

subplot(3,2,3)
hold on
plot(T,SMA_L_database.sigma(1:length(T)),'LineWidth',2,'color',color_l)
plot(T,SMA_R_database.sigma(1:length(T)),'LineWidth',2,'color',color_r)
legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Stress (Pa)','Interpreter','LaTex')

subplot(3,2,4)
hold on
plot(T,SMA_L_database.MVF(1:length(T)),'LineWidth',2,'color',color_l)
plot(T,SMA_R_database.MVF(1:length(T)),'LineWidth',2,'color',color_r)
legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('$\xi$','Interpreter','LaTex')

subplot(3,2,5)
hold on
plot(T,SMA_L_database.eps_t(1:length(T)),'LineWidth',2,'color',color_l)
plot(T,SMA_R_database.eps_t(1:length(T)),'LineWidth',2,'color',color_r)
legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Transformation strain','Interpreter','LaTex')

subplot(3,2,6)
hold on
plot(T,SMA_L_database.E(1:length(T)),'LineWidth',2,'color',color_l)
plot(T,SMA_R_database.E(1:length(T)),'LineWidth',2,'color',color_r)
legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Young Modulus (Pa)','Interpreter','LaTex')

%% Phase Diagram
figure
hold on
plot(SMA_L_database.T(1:length(T)), ...
     SMA_L_database.sigma(1:length(T))/1e6, ...
     'LineWidth',2,'color',color_l, ...
     'DisplayName', 'Left leg')
plot(SMA_R_database.T(1:length(T)), ...
     SMA_R_database.sigma(1:length(T))/1e6, ...
     'LineWidth',2,'color',color_r, ...
     'DisplayName', 'Right leg')

xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Transformation strain','Interpreter','LaTex')
phase_diagram(P, max([max(SMA_L_database.sigma), max(SMA_R_database.sigma)]))
end

