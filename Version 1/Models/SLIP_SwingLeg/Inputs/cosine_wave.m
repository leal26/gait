function [output] = cosine_wave(t, IP)
global period
global heat_switch
% frequency = 1/period;
frequency = IP.frequency;
amplitude = IP.amplitude;
phase = IP.phase;

if isnan(heat_switch)
    output = IP.mean + amplitude;
else
    output = IP.mean + amplitude*cos(2*pi*frequency*(t-heat_switch) + 2*pi*phase);
end
end
