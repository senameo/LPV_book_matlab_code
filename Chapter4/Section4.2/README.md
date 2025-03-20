This directory contains the example detailed in section 4.2 of Chapter 4  
It concerns the control of a non linear system using an Hinf /LPV control approaach, through a polytopic formulation  
%  
Main file: Section42.m  
It contains the modelling definition, control formulation and control solution  
%  
lmiHinfPolytope.m  
Yalmip/Matlab function that computes the LPV / Hinf controller solving the LMI problem   
%  
Init_simulations_section42.m   
It allows to run both simulation scenarii shown in section 4.2 of Chapter 4  
   
It is imporant to note, that, while this is not detailed in the text , the simulation scenario includes an inout disturbance of magnitude 1,   which acts:
- at time t=15 sec for case 1  
- at time t=20 sec for case 2  
  

