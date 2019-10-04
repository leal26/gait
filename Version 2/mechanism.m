function [displacement] = mechanism(strain, length_arm, length_sma_original)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
length_leg = 2*sqrt(length_arm.^2 - ((1-strain).*length_sma_original).^2/4);
displacement = 1 - length_leg;
end

