function [t,y,teOUT,yeOUT,ieOUT] = ode_history_dependent(F_ty, h, t, y_initial, Events, OutputFcn)
%4th order Runge-Kutta integration routine for history dependent functions
% F_ty()
%Input custom values and custom first-order differential equation
% h: Step_Value = input('Enter Step Value: ')
% Routine starts here
% t=t_span(1):h:t_span(2);
y=zeros(length(y_initial),length(t));
y(:,1)=y_initial;
prev_teOUT = [-1 -1 -1 -1 -1 -1];
done = false;
if nargin(F_ty) == 2

    for i=1:(length(t)-1)
        % Update data for output
        OutputFcn(t(i),y(:,i),[]);
        % Calculate next value
        y(:,i+1) = next_y(F_ty, y, t, h, i);
        % Determine if an event happened
        [teOUT,~,~] = Events(0,y(:,i+1));
        inflections = sign(prev_teOUT).*sign(teOUT);
        if i>1
            for j=1:length(inflections)
                if inflections(j) <= 0
                    done = true;
                    break
                end
            end
        end
        prev_teOUT = teOUT;
        if done
            y = y(:,1:i);
            t = t(:,1:i);
            break
        end
    end
elseif nargin(F_ty) == 3
    for i=1:(length(t)-1)
        y(:,i+1) = next_y(F_ty, y, t, h, i);
        [teOUT,~,~] = Events(0,y(:,i+1));
        inflections = sign(prev_teOUT).*sign(teOUT);
        for j=1:length(inflections)
            if inflections(j) <= 0
                done = true;
                break
            end
        end
        prev_teOUT = teOUT;
        if done
            y = y(:,1:i);
            t = t(:,1:i);
            break
        end
    end
end

yeOUT = y(:,end);
teOUT = t(end);
if i~= length(t)-1
    ieOUT = j;
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

function [y_i1] = next_y(F_ty, y, t, h, i)
    k1 = F_ty(t(i),y(:,i));
    k2 = F_ty(t(i)+0.5*h,y(:,i)+0.5*h*k1);
    k3 = F_ty((t(i)+0.5*h),(y(:,i)+0.5*h*k2));
    k4 = F_ty((t(i)+h),(y(:,i)+k3*h));

    y_i1 = y(:,i) + (1/6)*(k1+2*k2+2*k3+k4)*h;
end