function [] = sma_plotting(T, SMA_L, SMA_R, te_all)
global SMA_L_database
global SMA_R_database
global heat_switch
color_r = [127/256,127/256,127/256];
color_l = [0/256,45/256,98/256];

%%
figure
subplot(3,2,1)
hold on
for i=1:length(te_all)
    if ~isnan(te_all(i))
        xline(te_all(i), '--');
    end
end
plot(T,SMA_L_database.eps(1:length(T)),'LineWidth',2,'color',color_l)
plot(T,SMA_R_database.eps(1:length(T)),'LineWidth',2,'color',color_r)
% legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Strain','Interpreter','LaTex')

subplot(3,2,2)
hold on
for i=1:length(te_all)
    if ~isnan(te_all(i))
        xline(te_all(i), '--');
    end
end
plot(T,SMA_L.T_function(T),'--','LineWidth',2,'color',color_l)
plot(T,SMA_R.T_function(T),'--','LineWidth',2,'color',color_r)
plot(T,SMA_L_database.T(1:length(T)),'LineWidth',2,'color',color_l)


plot(T,SMA_R_database.T(1:length(T)),'LineWidth',2,'color',color_r)

% legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Temperature (K)','Interpreter','LaTex')

subplot(3,2,3)
hold on
for i=1:length(te_all)
    if ~isnan(te_all(i))
        xline(te_all(i), '--');
    end
end
plot(T,SMA_L_database.sigma(1:length(T)),'LineWidth',2,'color',color_l)
plot(T,SMA_R_database.sigma(1:length(T)),'LineWidth',2,'color',color_r)
% legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Stress (Pa)','Interpreter','LaTex')

subplot(3,2,4)
hold on
plot(T,SMA_L_database.MVF(1:length(T)),'LineWidth',2,'color',color_l)
plot(T,SMA_R_database.MVF(1:length(T)),'LineWidth',2,'color',color_r)
% legend('Left leg','Right leg')
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
% legend('Left leg','Right leg')
xlabel('Stride Time $[\sqrt{l_o/g}]$','Interpreter','LaTex')
ylabel('Young Modulus (Pa)','Interpreter','LaTex')


%% Stress-strain
figure
hold on
grid on; box on
plot(SMA_L_database.eps(1:length(T)), ...
     SMA_L_database.sigma(1:length(T))/1e6, ...
     'LineWidth',2,'color',color_l, ...
     'DisplayName', 'Left leg')
plot(SMA_R_database.eps(1:length(T)), ...
     SMA_R_database.sigma(1:length(T))/1e6, ...
     'LineWidth',2,'color',color_r, ...
     'DisplayName', 'Right leg')

margin = 0.1;
x_max = max([max(SMA_R_database.eps(1:length(T))), max(SMA_L_database.eps(1:length(T)))]);
x_min = min([min(SMA_R_database.eps(1:length(T))), min(SMA_L_database.eps(1:length(T)))]);
y_max = max([max(SMA_R_database.sigma(1:length(T))), max(SMA_L_database.sigma(1:length(T)))])/1e6;
y_min = min([min(SMA_R_database.sigma(1:length(T))), min(SMA_L_database.sigma(1:length(T)))])/1e6;
hx =  x_max - x_min;
hy = y_max - y_min;
xlim([x_min, x_max + margin*hx]);
ylim([y_min, y_max + margin*hy]);
xlabel('$\epsilon$','Interpreter','LaTex', ...
       'FontName','Times New Roman','fontsize', 14)
ylabel('$\sigma$ (MPa)','Interpreter','LaTex', ...
       'FontName','Times New Roman','fontsize', 14)
legend('Location','best')

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
phase_diagram(SMA_L, max([max(SMA_L_database.sigma), max(SMA_R_database.sigma)]))
end

