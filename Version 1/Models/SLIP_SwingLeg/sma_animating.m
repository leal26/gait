function [] = sma_animating(zr, zl)
global SMA_L_database
global SMA_R_database
global active_leg
persistent contStateIndices  systParamIndices discStateIndices
delete(gca)
subplot(1,2,2)
box on

% plot(SMA_R_database.eps(1:length(T)), ...
%      SMA_R_database.sigma(1:length(T))/1e6, ...
%      'LineWidth',2,'color',color_r, ...
%      'DisplayName', 'Right leg')

colorMap = [linspace(162,0,256)', linspace(20,68,256)', linspace(47,128,256)']/256;
colormap(colorMap);
margin = 0.1;

xlim = [0, 13];
ylim = [0, 155];

set(gca, 'Xlim', xlim, 'YLim', ylim, 'FontName','Times New Roman','fontsize', 14)
xlabel('Strain (0.001 m/m)', ...
       'FontName','Times New Roman','fontsize', 14)
ylabel('Stress (MPa)', ...
       'FontName','Times New Roman','fontsize', 14)
title('Active leg: Right leg', 'FontName','Times New Roman','fontsize', 14)
h = colorbar;
caxis([0 20])
set(h,'YTick', [0 20], 'FontName','Times New Roman','fontsize', 14)
ylabel(h,'Martensitic volume fraction (%)', 'FontName','Times New Roman','fontsize', 14, 'Rotation', 270)
colorMap = [linspace(162,0,256)', linspace(20,68,256)', linspace(47,128,256)']/256;
colormap(colorMap);            

if isempty(SMA_R_database)

    title('Active leg: Right    Function: Actuator')
else
    T = SMA_R_database.t;
    n = length(SMA_R_database.t(~isnan(SMA_R_database.t())));

    if zr == 2
        title('Active leg: Right    Function: Actuator')
        x = 1000*SMA_R_database.eps(1:n)';
        y = SMA_R_database.sigma(1:n)'/1e6;
        z = zeros(size(x));
        col = 100*SMA_R_database.MVF(1:length(T))';

        surf = surface([x(1:n);x(1:n)],[y(1:n);y(1:n)],[z(1:n);z(1:n)],[col(1:n);col(1:n)],...
            'facecol','no',...
            'edgecol','interp',...
            'linew',2); 

    elseif zl == 2
            title('Active leg: Left    Function: Brake')
        x = 1000*SMA_L_database.eps(1:n)';
        y = SMA_L_database.sigma(1:n)'/1e6;
        z = zeros(size(x));
        col = 100*SMA_L_database.MVF(1:length(T))';

        surf = surface([x(1:n);x(1:n)],[y(1:n);y(1:n)],[z(1:n);z(1:n)],[col(1:n);col(1:n)],...
            'facecol','no',...
            'edgecol','interp',...
            'linew',2); 

    else
        title('Active leg: None    Function: None')
    end
end

end
