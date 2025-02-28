function [] = phase_space(Y)

%% Phase diagram
figure
margin = 0.1;
labels = {'$x$', '$\dot{x}$', ...
          '$y$', '$\dot{y}$', ...
          '$\phi_l$', '$\dot{\phi}_l$', ...
          '$\phi_r$', '$\dot{\phi}_r$'};
for i=1:4
    ii = 1+2*(i-1);
    jj=ii+1;
    subplot(2,2,i)
    plot(Y(ii,:), Y(jj,:), 'LineWidth',2)
    grid on; box on
    hx = max(Y(ii,:)) - min(Y(ii,:));
    xlim([min(Y(ii,:)) - margin*hx, max(Y(ii,:)) + margin*hx]);
    hy = max(Y(jj,:)) - min(Y(jj,:));
    ylim([min(Y(jj,:)) - margin*hy, max(Y(jj,:)) + margin*hy]);
    xlabel(labels(ii),'Interpreter','LaTex', 'FontSize', 14);
    ylabel(labels(jj),'Interpreter','LaTex', 'FontSize', 14)
end

