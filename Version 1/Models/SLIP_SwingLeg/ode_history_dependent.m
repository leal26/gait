function [t,y,teOUT,yeOUT,ieOUT] = ode_history_dependent(F_ty, h, t, y_initial, Events, OutputFcn)
%4th order Runge-Kutta integration routine for history dependent functions
% F_ty()
%Input custom values and custom first-order differential equation
% h: Step_Value = input('Enter Step Value: ')
% Routine starts here
% t=t_span(1):h:t_span(2);
global active_leg
global SMA_R_database
global SMA_L_database
tol = -1e7;

y=zeros(length(y_initial),length(t));
y(:,1)=y_initial;
prev_teOUT = [-1 -1 -1 -1 -1 -1];
done = false;
for i=1:(length(t)-1)
    % Update data for output
    OutputFcn(t(i),y(:,i),[]);
    % Calculate next value
    y(:,i+1) = next_y(F_ty, y, t, h, i, true);
    % Determine if an event happened
    [teOUT,~,~] = Events(0,y(:,i+1));
    inflections = sign(prev_teOUT).*sign(teOUT);

    if i>1
        for j=1:length(inflections)
            if inflections(j) <= 0
                if i~= length(t)-1
                    done = true;
                    break
                end
            end
         end
        if (SMA_R_database.sigma(SMA_R_database.index)<tol) || (SMA_L_database.sigma(SMA_L_database.index)<tol) || y(2,i+1) < 0
%             disp(SMA_R_database.sigma(SMA_R_database.index))
%             disp(SMA_L_database.sigma(SMA_L_database.index))
            done = true;
            ieOUT = [];
        end
    end
    prev_teOUT = teOUT;
    if done
        y = y(:,1:i);
        t = t(:,1:i);
        break
    end
end

yeOUT = y(:,end);
teOUT = t(end);

try
   isempty(ieOUT);
catch
    if i~= length(t)-1
       ieOUT = j;
    end
end

end

function [y_i1] = next_y(F_ty, y, t, h, i, store)
global store_flag
    if isempty(store)
        k1 = F_ty(t(i),y(:,i));
    else
        store_flag = true;
        k1 = F_ty(t(i),y(:,i));
        store_flag = false;
    end
    k2 = F_ty(t(i)+0.5*h,y(:,i)+0.5*h*k1);
    k3 = F_ty((t(i)+0.5*h),(y(:,i)+0.5*h*k2));
    k4 = F_ty((t(i)+h),(y(:,i)+k3*h));

    y_i1 = y(:,i) + (1/6)*(k1+2*k2+2*k3+k4)*h;
end