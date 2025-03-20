This repository contains some of the examples from my book:  
Linear Parameter-Varying Control: Theory and Application to Automotive Systems
Published by John Wiley & Sons, Inc., Hoboken, New Jersey.
ISBN: 9781394285952

<<<<<<< HEAD
This book is accompanied by a companion website:
www.wiley.com/go/sename/lpvcontrol  
The website includes GitHub Repository  
=======
This book is accompanied by a companion website:  
www.wiley.com/go/sename/lpvcontrol
The website includes GitHub Repository
>>>>>>> 639b6a08dd01eba475a2ec5d3bcd2465b6d8701c

It is worth noting that a certain number of generic functions can be re-sued for stability analysis, obseerver and control design. To name a few:  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
Stability analysis 
%  
LMI_ParameterDependentStability_grid.m  
Function that checks the parameter dependent stability of an autonomous LPV system xdot=A(rho)x using a Parameter Depdendent Lyapunov Function   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
Control design  
%  
lmiHinfStateFeedbackRobust_expstab.m  
Function that computes a robust state feedback controller for a polytopic system solving an LMI problem.   
%  
lmiHinfStateFeedbackPolytope.m  
Yalmip/Matlab function that computes a LPV / Hinf state feedback controller solving the LMI problem   
%  
lmiHinfPolytope.m  
Yalmip/Matlab function that computes the LPV / Hinf controller solving the LMI problem. Build by Charles Poussot-Vassal during his PhD studies at Grenoble/GIPSA-lab  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
Observer design  
%  
obs_LPV_pol_poleregion_hinf.m  
Function that compute an LPV polytopic observer with Hinfinity and pole placement constraints (defined as LMI region)   
%  
  