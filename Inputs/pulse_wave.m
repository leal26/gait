function [output] = pulse_wave(t, IP)

phase = IP.phase;
duty = IP.duty;
frequency = IP.frequency;
amplitude = IP.amplitude;

period = 1/frequency;
time = rem(t,period);

if (time >= phase*period) && (time < period*(duty + phase))
    output = amplitude;
else
    output = 0;
end
end

