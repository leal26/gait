length_sma_original = linspace(.3,1);
strain = linspace(0, 0.04);
[EPS,S] = meshgrid(strain ,length_sma_original);

length_arm = sqrt(S.^2+1)/2;
displacement = mechanism(EPS, length_arm, S);

figure
contourf(EPS,S,displacement,10)
c = colorbar;
c.Label.String = 'Displacement';
xlabel('SMA strain')
ylabel('Original SMA length')


figure
contourf(EPS,S,length_arm,10)
c = colorbar;
c.Label.String = 'Arm length';
xlabel('SMA strain')
ylabel('Original SMA length')