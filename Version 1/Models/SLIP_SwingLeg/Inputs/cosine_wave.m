function [output] = cosine_wave(t, IP)

phase = IP.phase;
duty = IP.duty;
frequency = IP.frequency;
amplitude = IP.amplitude;

period = 1/frequency;
time = rem(t,period);

output = IP.mean + amplitude*cos(2*3.1415*frequency*(t+phase));
end
