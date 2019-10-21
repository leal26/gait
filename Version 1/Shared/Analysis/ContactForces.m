
%Calculates the contact forces after simulation

function contactF = ContactForces(y, z, p, t, SMA_L, SMA_R)
global SMA_L_database
global SMA_R_database

lxForce = zeros(length(t),1);
lyForce = zeros(length(t),1);
rxForce = zeros(length(t),1);
ryForce = zeros(length(t),1);
xForce = zeros(length(t),1);
yForce = zeros(length(t),1);
time = zeros(length(t),1);

lLength =  zeros(length(t),1);
rLength =  zeros(length(t),1);
values = struct('x',xForce,'y',yForce,'lx',lxForce,'ly',lyForce,'rx',rxForce,'ry',ryForce);
contactF = struct('time',time,'values',values);

for i=1:length(t)
    
contactF.time(i) = t(i);    

switch z(1,i)
        case 1 %(flight = 1 & 3)
            switch z(3,i)
                     case 1 %flight
                            contactF.values.rx(i) = 0;  
                            contactF.values.ry(i) = 0;
                            contactF.values.lx(i) = 0;  
                            contactF.values.ly(i) = 0;
                            
                            contactF.values.x(i) = contactF.values.lx(i)+contactF.values.rx(i);  
                            contactF.values.y(i) = contactF.values.ly(i)+contactF.values.ry(i);  
                            lLength(i) = 1;
                            rLength(i) = 1;
                            
                           
                     case 2 %(right stance
                            l_legr = sqrt((y(1,i)-z(4,i))^2 + (y(3,i)-0)^2);
                            gamma_legr = atan2(z(4,i)-y(1,i), y(3,i)-0);
                            if isnan(p(4))
                                % eps = (p(2)-l_legr)/p(2);
                                % SMA_R.eps = eps*l_legr*20/1.138989e+04;
                                T = SMA_R_database.T(i);
                                sigma = SMA_R_database.sigma(i);
                                F_sma = sigma*2*3.1415*SMA_R.r^3/(3*SMA_R.R)/SMA_R.norm; 
                                f_springr=F_sma;
                            else
                                f_springr=(p(2) - l_legr) * p(4);
                            end
                            contactF.values.rx(i) = f_springr * -sin(gamma_legr);  
                            contactF.values.ry(i) = f_springr * +cos(gamma_legr);     
                            
                            contactF.values.lx(i) = 0;  
                            contactF.values.ly(i) = 0;             
             
                            contactF.values.x(i) = contactF.values.lx(i)+contactF.values.rx(i);  
                            contactF.values.y(i) = contactF.values.ly(i)+contactF.values.ry(i);
                            
                            lLength(i) = 1;
                            rLength(i) = l_legr;
             
            end             
        case 2 %(stance = 2)
            switch z(3,i)
                     case 1 %(flight = 4 & 6)  
                            l_legl = sqrt((y(1,i)-z(2,i))^2 + (y(3,i)-0)^2);
                            gamma_legl = atan2(z(2,i)-y(1,i), y(3,i)-0);
                            f_springl = (p(2) - l_legl) * p(4);
                            contactF.values.lx(i) = f_springl * -sin(gamma_legl);  
                            contactF.values.ly(i) = f_springl * +cos(gamma_legl);     
                            
                            contactF.values.rx(i) = 0;  
                            contactF.values.ry(i) = 0;             
             
                            contactF.values.x(i) = contactF.values.lx(i)+contactF.values.rx(i);  
                            contactF.values.y(i) = contactF.values.ly(i)+contactF.values.ry(i);
                            lLength(i) = l_legl;
                            rLength(i) = 1;
             
                      case 2 %(stance = 5)
                            l_legl = sqrt((y(1,i)-z(2,i))^2 + (y(3,i)-0)^2);
                            gamma_legl = atan2(z(2,i)-y(1,i), y(3,i)-0);
                            f_springl = (p(2) - l_legl) * p(4);
                            contactF.values.lx(i) = f_springl * -sin(gamma_legl);
                            contactF.values.ly(i) = f_springl * +cos(gamma_legl);
                            
                            l_legr = sqrt((y(1,i)-z(4,i))^2 + (y(3,i)-0)^2);
                            gamma_legr = atan2(z(4,i)-y(1,i), y(3,i)-0);
                            f_springr = (p(2) - l_legr) * p(4);
                            contactF.values.rx(i) = f_springr * -sin(gamma_legr);
                            contactF.values.ry(i) = f_springr * +cos(gamma_legr);
                            
                            contactF.values.x(i) = contactF.values.lx(i)+contactF.values.rx(i);  
                            contactF.values.y(i) = contactF.values.ly(i)+contactF.values.ry(i);
                            
                            lLength(i) = l_legl;
                            rLength(i) = l_legr;                            
             end
end

end

% figure(6)
% subplot(3,1,1)
% plot(contactF.time, contactF.values.lx, 'r')
% hold on
% plot(contactF.time, contactF.values.rx, 'g')
% hold on
% plot(contactF.time, contactF.values.x)
% legend('GRF x direction left','GRF x direction right','GRF x direction total')
% 
% subplot(3,1,2)
% plot(contactF.time, contactF.values.ly, 'r')
% hold on 
% plot(contactF.time, contactF.values.ry, 'g')
% hold on
% plot(contactF.time, contactF.values.y)
% legend('GRF y direction left','GRF y direction right','GRF y direction total')
% 
% a=length(z(1,:));
% b=1:1:a;
% c=length(z(3,:));
% d=1:1:c;
% subplot(3,1,3)
% plot(b,z(1,:))
% hold on
% plot(d,z(3,:),'g')
% legend('phase')

% h = figure(1);
% clf(h);
FigHandle = figure('Position', [100, 100, 400, 300]);
% plot(contactF.time, contactF.values.rx, 'color',[127/256,127/256,127/256], 'LineWidth',2)
hold on  % Right leg is in grey
plot(contactF.time, contactF.values.ry, 'color',[127/256,127/256,127/256], 'LineWidth',2)
% plot(contactF.time, contactF.values.lx, 'color',[0/256,45/256,98/256], 'LineWidth',2)
hold on  % Left leg is in blue
plot(contactF.time, contactF.values.ly, 'color',[0/256,45/256,98/256], 'LineWidth',2)
% hold on  % Left leg is in blue
% plot(contactF.time, contactF.values.ly+contactF.values.ry, 'color','k', 'LineWidth',2)
% plot(contactF.time, lLength);
% plot(contactF.time, rLength);
% plot(contactF.time, ones(size(contactF.time)));
legend('vGRF R','vGRF L');
box on;
axis([0 contactF.time(end) -0.5 max([contactF.values.ry; contactF.values.ly])*1.2]);
% set(FigHandle, 'Position', [100, 100, 1049, 895]);
end