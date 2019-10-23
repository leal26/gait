function [] = design_plots(matrix, y_periodic)

f = 100*matrix(:,4);
max_x = matrix(:,end-3);
max_y = matrix(:,end-2);
max_dy = matrix(:,end-1);
power = matrix(:,end);
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
scatter(power,max_x,[],f,'filled')
yline(max(y_periodic(1,:)),'linewidth',2)
xlabel('Specific Power','Interpreter','LaTex')
ylabel('$\max(x)$','Interpreter','LaTex')
h = colorbar;
ylabel(h,'Phase (%)')
set(gca,'FontName', 'Times New Roman','FontSize', 14)

subplot(3,1,2)
hold on
scatter(power,max_y,[],f,'filled')
yline(max(y_periodic(3,:)),'linewidth',2)
xlabel('Specific Power','Interpreter','LaTex')
ylabel('$\max(y)$','Interpreter','LaTex')
set(gca,'FontName', 'Times New Roman','FontSize', 14)

subplot(3,1,3)
hold on
scatter(power,max_dy,[],f,'filled')
yline(max(y_periodic(4,:)),'linewidth',2)
xlabel('Specific Power','Interpreter','LaTex')
ylabel('$\max(dy/dt)$','Interpreter','LaTex')
set(gca,'FontName', 'Times New Roman','FontSize', 14)

end

