close all
clear all
clc

SMA_density = 6450;
data = ["brake", "strut", "actuator", "mixed"];
labels = ["Brake", "Strut", "Actuator", "Mixed"];
color_brake = [153/256,153/256,153/256];
color_strut = [0/256,0/256,0/256];
color_actuator = [102/256,102/256,102/256];
color_mixed = [0/256,128/256,0/256];
colors = [color_brake; color_strut; color_actuator; color_mixed];
figure(1);
xlabel('Time ($\sqrt{l_o/g}$)','Interpreter','LaTex')
ylabel('Temperature ')
set(gca,'FontName','Times New Roman','fontsize', 14,'linewidth',1.15)
set(gca,'XMinorTick','on','YMinorTick','on')
set(gca,'ticklength',3*get(gca,'ticklength'))
xlim([0,2.02])
hold on

figure(2)
set(gca,'FontName','Times New Roman','fontsize', 14,'linewidth',1.15)
set(gca,'XMinorTick','off','YMinorTick','off')
hold on

figure(3)
xlabel('Strain (%)','Interpreter','LaTex')
ylabel('Stress  (MPa) ','Interpreter','LaTex')
set(gca,'FontName','Times New Roman','fontsize', 14,'linewidth',1.15)
set(gca,'XMinorTick','off','YMinorTick','off')
xlim([0,1.3191])
ylim([0,165])
hold on

figure(4)
xlabel('Time ($\sqrt{l_o/g}$)','Interpreter','LaTex')
ylabel('Velocity  ($\sqrt{gl_o}$) ','Interpreter','LaTex')
set(gca,'FontName','Times New Roman','fontsize', 14,'linewidth',1.15)
set(gca,'XMinorTick','off','YMinorTick','off')
xlim([0,2.82])
ylim([1.25,1.52])
hold on

figure(6)
set(gca,'FontName','Times New Roman','fontsize', 14,'linewidth',1.15)
set(gca,'XMinorTick','off','YMinorTick','off')
xlim([0,2.02])
hold on

for i=1:length(data)
    c = colors(i,:);
    load(data(i))
    disp(parameters)
    T = SMA_R_database.t;
    
    figure(1);
    plot(T, SMA_L_database.T(1:length(T)),'-','LineWidth',2,'color',c, 'DisplayName', 'Left ' + data(i))
    plot(T, SMA_R_database.T(1:length(T)),'--','LineWidth',2,'color',c, 'DisplayName', 'Right ' + data(i))
    
    figure(2)
    plot(SMA_L_database.T(1:length(T)), SMA_L_database.sigma(1:length(T))/1e6, '-','LineWidth',2,'color',c, 'DisplayName', 'Left ' + data(i))
    % plot(SMA_R_database.T(1:length(T)), SMA_R_database.sigma(1:length(T))/1e6,'--','LineWidth',2,'color',colors(i))
    
    figure(3)
    plot(100*SMA_L_database.eps(1:length(T)), SMA_L_database.sigma(1:length(T))/1e6, '-','LineWidth',2,'color',c, 'DisplayName', 'Left ' + data(i))
    % plot(100*SMA_R_database.eps(1:length(T)), SMA_R_database.sigma(1:length(T))/1e6,'--','LineWidth',2,'color',colors(i))
    
    figure(4)
    if strcmp(data(i), "mixed")
        plot(simRES.t, simRES.continuousStates(contStateIndices.dx,:), '--','LineWidth',2.74,'color',c, 'DisplayName', labels(i))
    else
        plot(simRES.t, simRES.continuousStates(contStateIndices.dx,:), '-','LineWidth',2.74,'color',c, 'DisplayName', labels(i))
    end
    figure(6)
    plot(SMA_R_database.t, SMA_L_database.MVF(1:length(T)), '-','LineWidth',2,'color',c, 'DisplayName', 'Left ' + data(i))
    plot(SMA_R_database.t, SMA_R_database.MVF(1:length(T)), '--','LineWidth',2,'color',c, 'DisplayName', 'Right ' + data(i))

end
figure(2)
phase_diagram(SMA_L, 165e6)

figure(1)
legend('Location','best')

figure(2)
legend('Location','best')

figure(3)
legend('Location','best')

figure(4)
legend('Location','best')