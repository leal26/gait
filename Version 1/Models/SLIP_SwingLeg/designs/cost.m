function output = cost(parameters, lb, ub,yCYC, zCYC, pCYC, contStateIndices)
    parameters = (ub-lb).*parameters + lb;
    disp(parameters)
    IP.mean = parameters(1);
    IP.amplitude = parameters(2);
    IP.phase = parameters(3);
    
    IP.frequency = sqrt(5)/2/pi;
    IP.mass = 10; % kg (used for normalizing)
    IP.gravity = 9.80665; % m/s2 (used for normalizing)
    SMA_density = 6450; %kg/m3
    [SMA_L, SMA_R] = define_SMA(IP, IP);
    SMA_R.phase = parameters(4);

    recOUTPUT = RecordStateCLASS();
    recOUTPUT.rate = 0.01;
    % Checking Austenite constraint
    success = true;
    if SMA_R.T_function(0) >= SMA_R.A_f
%         try
            [yOUT, zOUT, tOUT, recOUTPUT] = HybridDynamics(yCYC, zCYC, pCYC, SMA_L, SMA_R, recOUTPUT, simOptions);
            simRES = recOUTPUT.retrieve();
            plotStates = [ contStateIndices.x, contStateIndices.dx,contStateIndices.y, contStateIndices.dy, contStateIndices.phiL, contStateIndices.dphiL,contStateIndices.phiR, contStateIndices.dphiR];

            if min(simRES.continuousStates(contStateIndices.y,:)) > 0
                max_x = max(simRES.continuousStates(contStateIndices.x,:));
                av_dx = mean(simRES.continuousStates(contStateIndices.dx,end-300:end));
                av_dy = mean(simRES.continuousStates(contStateIndices.dy,end-300:end));
                power_R = calculate_specific_power(SMA_R_database.sigma(1:length(simRES.t)), ...
                                             SMA_R_database.eps(1:length(simRES.t)), ...
                                             SMA_density, recOUTPUT.rate, 1);
                power_L = calculate_specific_power(SMA_L_database.sigma(1:length(simRES.t)), ...
                                             SMA_L_database.eps(1:length(simRES.t)), ...
                                             SMA_density, recOUTPUT.rate, 1);
            else
                success = false;
            end
%         catch
%             success = false;
%         end
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
    
    output = -max_x;