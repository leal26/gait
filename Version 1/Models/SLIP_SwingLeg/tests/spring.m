function [gamma] = spring(r, ratio, G, k)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
R = r/ratio;
u = .5;
N = r^4*G/(4*R^3*k);
gamma = 3*r/(8*3.1415*N*R^2)*u;
disp(gamma)
% tau = G*gamma;
% F = 2*3.1415*r^3/(3*R)*tau;
end

