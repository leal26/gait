function [total_power] = calculate_specific_power(sigma, eps, density, rate, scale)
% If area=L0=1, intrinsic work is calculated
total_work = 0;

time = 0;
for i=1:length(sigma)-1
    if ~isnan(sigma(i+1)) && ~isnan(eps(i+1))
        delta_eps = (eps(i+1) - eps(i));
        av_sigma = (sigma(i+1) + sigma(i))/2.;
        total_work = total_work - delta_eps*av_sigma/density;
        time = time + rate*scale;
    end
end
total_power = total_work/time;