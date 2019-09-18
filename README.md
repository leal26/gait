# README #

This is the second implementation of the proposed bipedal model with passive swing leg motion.
Different from the gait creation framework we showed in Version1, we remove the discrete states 
from the system and finding a gait is equivalent to solving a boundary value problem (BVP).
Detailed explanation of this approach can be found in the following paper:
Gan, Zhenyu, Ziyuan Jiao, and C. David Remy. "On the Dynamic Similarity between Bipeds and Quadrupeds: 
a Case Study on Bounding." IEEE Robotics and Automation Letters (2018).


% ************************************
% ************************************
% ..\StoredFunctions:
% ZeroFunc_BipedApex.m (integrate until the stride time)
% ContinuationFun.m (find solution along branches)
% f_Continuation.m (define BVP)
% ************************************
% ..\Graphics:
% This folder includes all functions used to play animation of the bipedal 
% model.
% ************************************
% ************************************

Some of the identified solutions are stored in the folder called BipedalGaits.

Please run BipedSlipDynamics.m to see the results. Please note that the current terminal event is selected 
as the apex transition.
