r = linspace(0.001, 0.002);
ratio = linspace(1e-1, 0.5);

[r,ratio] = meshgrid(r,ratio);

G = 3.7427e+10/(2*(1+0.3));
k = 20*98.0665; % M=10kg, g=9.80665, l0=1
R = r./ratio;
u = .4;
N = r.^4*G./(4*R.^3*k);
gamma = 3*r./(8*3.1415*N.*R.^2)*u;

figure

contourf(r/1e-3,ratio,gamma)
set(gca, 'YScale', 'log')
set(gca,'FontName','Times New Roman','fontsize', 14)
xlabel('Wire radius (mm)', ...
       'FontName','Times New Roman','fontsize', 14)
ylabel('Wire radius/Spring Radius', ...
       'FontName','Times New Roman','fontsize', 14)
h = colorbar;
ylabel(h,'Strain', 'FontName','Times New Roman','fontsize', 14)