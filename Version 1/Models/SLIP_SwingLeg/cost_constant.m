function output = cost(parameters, lb, ub,yCYC, zCYC, pCYC, contStateIndices, force, store)
    global SMA_L_database
    global SMA_R_database
    global individuals
    global fitnesses
    global ind_index
    global active_leg
    parameters = (ub-lb).*parameters + lb;
    % disp(parameters)
    IP.mean = parameters(1);
    IP.amplitude = parameters(2);
    IP.phase = parameters(3);
    
    IP.frequency = parameters(5);% sqrt(5)/2/pi;
    IP.mass = 10; % kg (used for normalizing)
    IP.gravity = 9.80665; % m/s2 (used for normalizing)
    IP.active_leg = 'right';
    SMA_density = 6450; %kg/m3
    [SMA_L, SMA_R] = define_SMA(IP, IP);
    SMA_R.phase = parameters(4);
    if ~isempty(force)
        SMA_R.F_external = force;
    end
    simOptions.tIN = 0; 
    simOptions.tMAX = 40; 
    recOUTPUT = RecordStateCLASS();
    recOUTPUT.rate = 0.01;
    % Checking Austenite constraint
    success = true;
    if SMA_R.T_function(0) >= SMA_R.A_f
         try
            [yOUT, zOUT, tOUT, recOUTPUT] = HybridDynamics(yCYC, zCYC, pCYC, SMA_L, SMA_R, recOUTPUT, simOptions);
            simRES = recOUTPUT.retrieve();
            plotStates = [ contStateIndices.x, contStateIndices.dx,contStateIndices.y, contStateIndices.dy, contStateIndices.phiL, contStateIndices.dphiL,contStateIndices.phiR, contStateIndices.dphiR];

            if min(simRES.continuousStates(contStateIndices.y,:)) > 0
                max_x = max(simRES.continuousStates(contStateIndices.x,:));
                av_dx = mean(simRES.continuousStates(contStateIndices.dx,end-10:end));
                av_dy = mean(simRES.continuousStates(contStateIndices.dy,end-10:end));
                power_R = calculate_specific_power(SMA_R_database.sigma(1:length(simRES.t)), ...
                                             SMA_R_database.eps(1:length(simRES.t)), ...
                                             SMA_density, recOUTPUT.rate, 1);

                power_L = calculate_specific_power(SMA_L_database.sigma(1:length(simRES.t)), ...
                                             SMA_L_database.eps(1:length(simRES.t)), ...
                                             SMA_density, recOUTPUT.rate, 1);
            else
                success = false;
            end
        catch
            success = false;
        end
    else
        success = false;
    end
    if ~success
        max_x = -9999;
        av_dx = -9999;
        av_dy = -9999;
        power_L = -9999;
        power_R = -9999;
    end
    
    output = abs(21.7661-max_x);
    if max_x ~= -9999 && store
        individuals(ind_index,:) = parameters;
        fitnesses(ind_index,1) = max_x;
        fitnesses(ind_index,2) = av_dx;
        fitnesses(ind_index,3) = av_dy;
        fitnesses(ind_index,4) = power_L;
        fitnesses(ind_index,5) = power_R;
        
        ind_index = ind_index + 1;
        scatter(power_L, max_x,[],'b','filled')
        shg
    end