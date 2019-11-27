close all
clear all
clc

SMA_density = 6450;
data = ["brake", "strut","actuator"];
labels = ["Brake", "Strut","Actuator"];
color_brake = [153/256,153/256,153/256];
color_strut = [102/256,102/256,102/256];
color_actuator = [0/256,0/256,0/256];
colors = [color_brake; color_strut; color_actuator];
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
xlim([0,2.78])
ylim([1.3,1.52])
hold on

% figure(5)
% xlabel('Time ($\sqrt{l_o/g}$)','Interpreter','LaTex')
% ylabel('Instantaneous specific power ($\sqrt{gl_o^3}$) ','Interpreter','LaTex')
% set(gca,'FontName','Times New Roman','fontsize', 14,'linewidth',1.15)
% set(gca,'XMinorTick','off','YMinorTick','off')
% hold on

figure(6)
hold on

for i=1:length(data)
    c = colors(i,:);
    load(data(i))
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
    plot(simRES.t, simRES.continuousStates(contStateIndices.dx,:), '-','LineWidth',2,'color',c, 'DisplayName', labels(i))
    
    figure(6)
    plot(100*SMA_L_database.eps(1:length(T)), SMA_L_database.MVF(1:length(T)), '-','LineWidth',2,'color',c, 'DisplayName', 'Left ' + data(i))
%     figure(5)
%     scale = 1;
%     eps = SMA_L_database.eps;
%     sigma = SMA_L_database.sigma;
%     tspan = SMA_R_database.t;
%     Power = nan(size(eps));
%     for j=1:length(sigma)-1
%         if (~isnan(sigma(j)) && ~isnan(sigma(j+1))) && (~isnan(eps(j)) && ~isnan(eps(j+1)))
%             delta_eps = (eps(j+1) - eps(j));
%             av_sigma = (sigma(j+1) + sigma(j))/2.;
%             work = delta_eps*av_sigma/SMA_density;
%             time = (tspan(i+1)-tspan(i))*scale;
%             Power(j)=work/time;
%         end
%     end
%     plot(tspan, Power,'-','LineWidth',2,'color',c, 'DisplayName', 'Left ' + data(i))

%     eps = SMA_R_database.eps;
%     sigma = SMA_R_database.sigma;
%     tspan = SMA_R_database.t;
%     Power = nan(size(eps));
%     for j=1:length(sigma)-1
%         if (~isnan(sigma(j)) && ~isnan(sigma(j+1))) && (~isnan(eps(j)) && ~isnan(eps(j+1)))
%             delta_eps = (eps(j+1) - eps(j));
%             av_sigma = (sigma(j+1) + sigma(j))/2.;
%             work = delta_eps*av_sigma/SMA_density;
%             time = (tspan(i+1)-tspan(i))*scale;
%             Power(j)=work/time;
%         else
%             Power(j) = 0;
%         end
%     end
%     plot(tspan, Power,'--','LineWidth',2,'color',c, 'DisplayName', 'Right ' + data(i))
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

figure(5)
legend('Location','best')