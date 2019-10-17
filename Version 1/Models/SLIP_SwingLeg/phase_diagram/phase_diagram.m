function [] = phase_diagram(P, max_sigma)
global experiment

if nargin < 2
    max_sigma = 0;
    fields = fieldnames(experiment(1));
    for i=1:length(fields)
        field = char(fields(i));
        if max(experiment(3).(field)) > max_sigma
            max_sigma = max(experiment(3).(field));
        end
    end
end
sigma = linspace(0, max_sigma);

% Current transformation strain at calibration stress
H_cur_cal = H_cursolver(P.sig_cal, P.sig_crit,P.k,P.H_min,P.H_sat);

% Partial Derivative of H_cur at calibration stress (dH_cur)
dH_cur=partial_Hcur_sigma(P.sig_cal,P.sig_crit,P.k,P.H_sat,P.H_min);

%% Transformation Parameters (structure: TP)
TP.rho_delta_s0 = (-2*(P.C_M*P.C_A)*(H_cur_cal+P.sig_cal*dH_cur+P.sig_cal*(1/P.E_M-1/P.E_A)))/(P.C_M+P.C_A);
TP.D = ((P.C_M-P.C_A)*(H_cur_cal+P.sig_cal*dH_cur+P.sig_cal*(1/P.E_M-1/P.E_A)))/((P.C_M+P.C_A)*(H_cur_cal+...
    P.sig_cal*dH_cur));
TP.a1 = TP.rho_delta_s0*(P.M_f-P.M_s);
TP.a2 = TP.rho_delta_s0*(P.A_s-P.A_f);
TP.a3 = -TP.a1/4*(1+1/(P.n1+1)-1/(P.n2+1))+TP.a2/4*(1+1/(P.n3+1)-1/(P.n4+1));
TP.rho_delta_u0 = TP.rho_delta_s0/2*(P.M_s+P.A_f);
TP.Y_0_t = TP.rho_delta_s0/2*(P.M_s-P.A_f)-TP.a3;


%% Determine Stress vs. Temperature PHase Diagram

% Arrays of output variables
% T_fwd_0: Temperature array for forward transformation at MVF=0
T_fwd_0 = zeros(size(sigma));

% T_fwd_1: Temperature array for forward transformation at MVF=1
T_fwd_1 = zeros(size(sigma));

% T_rev_0: Temperature array for reverse transformation at MVF=0
T_rev_0 = zeros(size(sigma));

% T_rev_0: Temperature array for reverse transformation at MVF=1
T_rev_1 = zeros(size(sigma));


for i = 1:length(sigma)
    [T_fwd_0(i)]=Forward_Transformation(sigma(i),0,P,TP);
    [T_fwd_1(i)]=Forward_Transformation(sigma(i),1,P,TP);
    [T_rev_0(i)]=Reverse_Transformation(sigma(i),0,P,TP);
    [T_rev_1(i)]=Reverse_Transformation(sigma(i),1,P,TP);
end

box on 
hold on
plot(T_fwd_0,sigma/(1e6),'b', 'LineWidth',2, 'DisplayName', '\Phi_{fwd, \xi = 0}')
plot(T_fwd_1,sigma/(1e6),'--b', 'LineWidth',2, 'DisplayName', '\Phi_{fwd, \xi = 1}')
plot(T_rev_0,sigma/(1e6),'--r', 'LineWidth',2, 'DisplayName', '\Phi_{rev, \xi = 0}')
plot(T_rev_1,sigma/(1e6),'r', 'LineWidth',2, 'DisplayName', '\Phi_{rev, \xi = 1}')
xlabel('Temperature (K)')
ylabel('Stress (MPa)')
% title('SMA Model Phase Diagram')
set(gca,'FontName','Times New Roman','fontsize', 14,'linewidth',1.15)
set(gca,'XMinorTick','on','YMinorTick','on')
set(gca,'ticklength',3*get(gca,'ticklength'))
legend('Location','best')
end
