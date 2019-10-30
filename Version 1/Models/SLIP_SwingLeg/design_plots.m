function [] = design_plots(matrix, y_periodic)

phase_L = 100*matrix(:,3);

max_x = matrix(:,end-4);
av_dx = matrix(:,end-3);
av_dy = matrix(:,end-2);
power_L = matrix(:,end-1);
power_R = matrix(:,end);
    
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
scatter(power_R,max_x,[],phase_L,'filled');
%yline(max(y_periodic(1,:)),'linewidth',2);
xlabel('Specific Power, right leg','Interpreter','LaTex');
ylabel('$\max(x)$','Interpreter','LaTex');
h = colorbar;
ylabel(h,'Phase, left leg(%)');
set(gca,'FontName', 'Times New Roman','FontSize', 14);

subplot(3,1,2)
hold on
scatter(power_R,av_dx,[],phase_L,'filled');
% yline(max(y_periodic(3,:)),'linewidth',2);
xlabel('Specific Power, right leg','Interpreter','LaTex');
ylabel('Average Speed','Interpreter','LaTex');
set(gca,'FontName', 'Times New Roman','FontSize', 14);

subplot(3,1,3)
hold on
scatter(max_x,av_dx,[],phase_L,'filled');
%xline(max(y_periodic(4,:)),'linewidth',2);
xlabel('$\max(x)$','Interpreter','LaTex');
ylabel('Average Speed','Interpreter','LaTex');
set(gca,'FontName', 'Times New Roman','FontSize', 14);

end

