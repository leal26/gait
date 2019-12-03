function [] = design_plots(matrix, parameters, y_periodic)

phase_L = 100*matrix(:,3);

max_x = matrix(:,end-5);
av_dx = matrix(:,end-4);
av_dy = matrix(:,end-3);
power_L = matrix(:,end-2);
power_R = matrix(:,end-1);
periodicity = matrix(:,end);
color_l = [127/256,127/256,127/256];
color_r = [0/256,45/256,98/256];

%%
figure
% subplot(2,2,1)
% hold on
% scatter(max_x,max_y,[],f,'filled')
% xlabel('$max(x)$','Interpreter','LaTex')
% ylabel('$max(y)$','Interpreter','LaTex')

subplot(3,1,1)
hold on
scatter(power_R,max_x,[],parameters,'filled');
%yline(max(y_periodic(1,:)),'linewidth',2);
xlabel('Specific Power, right leg','Interpreter','LaTex');
ylabel('$\max(x)$','Interpreter','LaTex');
h = colorbar;
ylabel(h,'Phase, left leg(%)');
set(gca,'FontName', 'Times New Roman','FontSize', 14);

subplot(3,1,2)
hold on
scatter(power_R,av_dx,[],parameters,'filled');
% yline(max(y_periodic(3,:)),'linewidth',2);
xlabel('Specific Power, right leg','Interpreter','LaTex');
ylabel('Average Speed','Interpreter','LaTex');
set(gca,'FontName', 'Times New Roman','FontSize', 14);

subplot(3,1,3)
hold on
scatter(max_x,av_dx,[],parameters,'filled');
%xline(max(y_periodic(4,:)),'linewidth',2);
xlabel('$\max(x)$','Interpreter','LaTex');
ylabel('Average Speed','Interpreter','LaTex');
set(gca,'FontName', 'Times New Roman','FontSize', 14);

figure

power_L(periodicity == 9990) = NaN;
power_R(periodicity == 9990) = NaN;
periodicity(periodicity == 9990) = NaN;
periodicity(periodicity > 3) = NaN;
disp(max(periodicity))
disp(min(periodicity))
scatter(power_L, power_R,[], [0/256,0/256,0/256], 'filled');
%xline(max(y_periodic(4,:)),'linewidth',2);
xlabel('Power left','Interpreter','LaTex');
ylabel('Power right','Interpreter','LaTex');
set(gca,'FontName', 'Times New Roman','FontSize', 14);
xline(0);
yline(0);
xlim([-70, 30])
ylim([-90, 10])
% colorbar;
end

