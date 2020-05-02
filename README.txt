# README #
This directory contains 'Codes' pertinent to the paper 'Phase transformation-driven artificial muscle mimics the multi-functionality of avian wing muscle'. Excluding legacy software, the code was developed by Pedro B.C. Leal.

The provided Matlab codes do not require additional dependencies. This work leverages the following existing codes:
	- Compass gait model developed by Zhenyu Gan (2018) and David C. Remy (2011)
	- 1D SMA Constitutive Model developed by Darren J. Hartl (2012), Cullen Nauck, and Ediwn Peraza-Hernandez (2016)

The three main codes are:
	- Models\SLIP_SwingLeg\Main_SLIP.m
		Evaluates one stimulus combination on the compass gait model fully coupled with an SMA constitutive model
	- Models\SLIP_SwingLeg\DOE.m
		Evaluates multiple stimuli combination on the compass gait model fully coupled with an SMA constitutive model
	- Models\SLIP_SwingLeg\_results\prepping_images.m
		Combines the calculate state variables for various stimuli combination in one plot
		
All other directories host files necessary for the simulation of the compass gait with artificial muscles.

For more information, please contact Pedro Leal at leal26@tamu.edu or leal2608@gmail.com.



