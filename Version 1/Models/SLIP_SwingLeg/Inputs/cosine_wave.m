function [output] = cosine_wave(t, IP)

frequency = IP.frequency;
amplitude = IP.amplitude;
phase = IP.phase;

output = IP.mean + amplitude*cos(2*3.1415*frequency*(t+phase));
end
