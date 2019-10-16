function [t, y] = ode_history_dependent(F_ty, n_steps, t_span, y_initial)
%4th order Runge-Kutta integration routine for history dependent functions
% F_ty()
%Input custom values and custom first-order differential equation
% h: Step_Value = input('Enter Step Value: ')
% Routine starts here
if nargin(F_ty) == 2
    h=(t_span(2)-t_span(1))/n_steps;
    t=t_span(1):h:t_span(2);
    y=zeros(length(y_initial),length(t));
    y(:,1)=y_initial;
    for i=1:(length(t)-1)

        k1 = F_ty(t(i),y(:,i));
        k2 = F_ty(t(i)+0.5*h,y(:,i)+0.5*h*k1);
        k3 = F_ty((t(i)+0.5*h),(y(:,i)+0.5*h*k2));
        k4 = F_ty((t(i)+h),(y(:,i)+k3*h));


        y(:,i+1) = y(:,i) + (1/6)*(k1+2*k2+2*k3+k4)*h;
    end
elseif nargin(F_ty) == 3
    h=(t_span(2)-t_span(1))/n_steps;
    t=t_span(1):h:t_span(2);
    y=zeros(length(y_initial),length(t));
    y(:,1)=y_initial;
    for i=1:(length(t)-1)

        k1 = F_ty(t(i),y(:,i),t(i-1));
        k2 = F_ty(t(i)+0.5*h,y(:,i)+0.5*h*k1);
        k3 = F_ty((t(i)+0.5*h),(y(:,i)+0.5*h*k2));
        k4 = F_ty((t(i)+h),(y(:,i)+k3*h));


        y(:,i+1) = y(:,i) + (1/6)*(k1+2*k2+2*k3+k4)*h;
    end
end
%Checking custom routine and MATLAB function ode45
% [t_c,y_c] = ode45(F_ty,t,y_initial);
% %Plot in one figure
% figure(1)
% plot(t,y)
% hold on
% plot(t_c,y_c, 'o')
% hold off
% y_c2=y_c';
% %Calculates error between custom runge-kutta routine and MATLAB function
% %ode45
% disp('Error between Runge-Kutta and ode45')
% err = immse(y,y_c2)
end

