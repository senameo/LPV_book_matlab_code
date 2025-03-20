This directory contains Matlab and Simulink files to run the example provided in section 7.3 of Chapter7.
The files to be considered are:
%%
Main: Section73.m  
Contains the whole program
%%
InputFilter4Polytopic.m
Matlab function to apply an input filtering method: needed to consider the polytopic approach
%%
lmiHinfPolytope.m
Yalmip/Matlab function that computes the LPV / Hinf controller solving the LMI problem 
%%
Simulations_section73.slx
Simulnik function that executes the silumaiton scenarios (the scenarios are chosen in the main file)