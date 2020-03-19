function [] = sma_animate(SMA_L, SMA_R, te_all)
global SMA_L_database
global SMA_R_database
global heat_switch
color_r = [127/256,127/256,127/256];
color_l = [0/256,45/256,98/256];

T = SMA_R_database.t;
TT = linspace(0, max(T), 1000);

indexes_r = [1,24,40,56,99];
sigma_r = SMA_R_database.sigma(indexes_r)/1e6;
eps_r = SMA_R_database.eps(indexes_r);
MVF_r = SMA_R_database.MVF(indexes_r);
% scatter(eps_r, sigma_r, [], MVF_r, 'filled', 'DisplayName', 'Right leg')


indexes_l = [135,159,182,203];
sigma_l = SMA_L_database.sigma(indexes_l)/1e6;
eps_l = SMA_L_database.eps(indexes_l);
MVF_l = SMA_L_database.MVF(indexes_l);
% scatter(eps_l, sigma_l, [], MVF_l, 'filled', 'DisplayName', 'Left leg')

% figure
% hold on
% box on
% 
% x = SMA_L_database.eps(1:length(T))';
% y = SMA_L_database.sigma(1:length(T))'/1e6;
% z = zeros(size(x));
% col = SMA_L_database.MVF(1:length(T))';  % This is the color, vary with x in this case.
% 
% surface([x;x],[y;y],[z;z],[col;col],...
%         'facecol','no',...
%         'edgecol','interp',...
%         'linew',2);
% scatter(eps_l, sigma_l, [], MVF_l, 'filled')
% colorbar
% caxis([0 0.2])
% 
margin = 0.1;
% x_max = max([max(SMA_R_database.eps(1:length(T))), max(SMA_L_database.eps(1:length(T)))]);
% x_min = min([min(SMA_R_database.eps(1:length(T))), min(SMA_L_database.eps(1:length(T)))]);
% y_max = max([max(SMA_R_database.sigma(1:length(T))), max(SMA_L_database.sigma(1:length(T)))])/1e6;
% y_min = min([min(SMA_R_database.sigma(1:length(T))), min(SMA_L_database.sigma(1:length(T)))])/1e6;
% hx =  x_max - x_min;
% hy = y_max - y_min;
% xlim([x_min, x_max + margin*hx]);
% ylim([y_min, y_max + margin*hy]);
% xlabel('$\epsilon$','Interpreter','LaTex', ...
%        'FontName','Times New Roman','fontsize', 14)
% ylabel('$\sigma$ (MPa)','Interpreter','LaTex', ...
%        'FontName','Times New Roman','fontsize', 14)
   
h = figure('position', [100 100 600 600], 'Renderer','Painters','Color','w')
hold on
box on

% plot(SMA_R_database.eps(1:length(T)), ...
%      SMA_R_database.sigma(1:length(T))/1e6, ...
%      'LineWidth',2,'color',color_r, ...
%      'DisplayName', 'Right leg')

colorMap = [linspace(162,0,256)', linspace(20,68,256)', linspace(47,128,256)']/256;
colormap(colorMap);

x = SMA_R_database.eps(1:length(T))';
y = SMA_R_database.sigma(1:length(T))'/1e6;
z = zeros(size(x));
col = SMA_R_database.MVF(1:length(T))';  % This is the color, vary with x in this case.

x_max = max([max(SMA_R_database.eps(1:length(T))), max(SMA_L_database.eps(1:length(T)))]);
x_min = min([min(SMA_R_database.eps(1:length(T))), min(SMA_L_database.eps(1:length(T)))]);
y_max = max([max(SMA_R_database.sigma(1:length(T))), max(SMA_L_database.sigma(1:length(T)))])/1e6;
y_min = min([min(SMA_R_database.sigma(1:length(T))), min(SMA_L_database.sigma(1:length(T)))])/1e6;
hx =  x_max - x_min;
hy = y_max - y_min;
xlim([x_min, x_max + margin*hx]);
ylim([y_min, y_max + margin*hy]);

curve = animatedline();
set(gca, 'Xlim', 1000*[x_min, x_max + margin*hx], 'YLim', [y_min, y_max + margin*hy])
xlabel('Strain (0.001 m/m)','Interpreter','LaTex', ...
       'FontName','Times New Roman','fontsize', 14)
ylabel('Stress (MPa)','Interpreter','LaTex', ...
       'FontName','Times New Roman','fontsize', 14)

colorbar
caxis([0 0.2])

n = length(SMA_R_database.t(~isnan(SMA_R_database.t())));
surf = surface([x(1:1);x(1:1)],[y(1:1);y(1:1)],[z(1:1);z(1:1)],[col(1:1);col(1:1)],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',4);
filename = 'video_test.mp4';

% Capture the plot as an image 
frame = getframe(h); 
im = frame2im(frame); 
[imind,cm] = rgb2ind(im,256); 
% Write to the GIF File 
imwrite(imind,cm,filename,'mp4', 'Loopcount',inf); 

for i=2:(n-1)
    t = SMA_R_database.t(i);
    if t <= te_all(1)
        title('Active leg: Right')
        x = 1000*SMA_R_database.eps(1:n)';
        y = SMA_R_database.sigma(1:n)'/1e6;
        z = zeros(size(x));
        col = SMA_R_database.MVF(1:length(T))';
    elseif (t > te_all(1)) && (t < te_all(3))
            title('Active leg: None')
    else
            title('Active leg: Left')
        x = 1000*SMA_L_database.eps(1:n)';
        y = SMA_L_database.sigma(1:n)'/1e6;
        z = zeros(size(x));
        col = SMA_L_database.MVF(1:length(T))';
    end
    delete(surf)
    if (t <= te_all(1)) || (t >= te_all(3))
        surf = surface([x(1:i+1);x(1:i+1)],[y(1:i+1);y(1:i+1)],[z(1:i+1);z(1:i+1)],[col(1:i+1);col(1:i+1)],...
                'facecol','no',...
                'edgecol','interp',...
                'linew',2); 
    end
    drawnow
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256);
    imwrite(imind,cm,filename,'mp4','WriteMode','append'); 
end

% xlim([0 0.013])
% ylim([0 150])

% legend('Location','best')

end
