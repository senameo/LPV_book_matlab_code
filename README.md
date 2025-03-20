# Linear Parameter-Varying Control: Examples Repository

This repository contains examples from the book:

**Linear Parameter-Varying Control: Theory and Application to Automotive Systems**  
*Published by John Wiley & Sons, Inc., Hoboken, New Jersey.*  
**ISBN:** 9781394285952  

## Companion Website
This book is accompanied by a companion website:  
[www.wiley.com/go/sename/lpvcontrol](www.wiley.com/go/sename/lpvcontrol)  
The website includes a GitHub repository with additional resources.

---

## Available Functions
Several generic functions can be reused for stability analysis, observer design, and control design. Below is a categorized list of these functions:

### 🔹 Stability Analysis
- **LMI_ParameterDependentStability_grid.m**  
  Checks the parameter-dependent stability of an autonomous LPV system \( \dot{x} = A(\rho)x \) using a Parameter-Dependent Lyapunov Function.

### 🔹 Control Design
- **lmiHinfStateFeedbackRobust_expstab.m**  
  Computes a robust state feedback controller for a polytopic system by solving an LMI problem.
- **lmiHinfStateFeedbackPolytope.m**  
  YALMIP/Matlab function that computes an LPV/H∞ state feedback controller by solving an LMI problem.
- **lmiHinfPolytope.m**  
  YALMIP/Matlab function that computes an LPV/H∞ controller by solving the LMI problem. Originally developed by Charles Poussot-Vassal during his PhD at Grenoble/GIPSA-lab.

### 🔹 Observer Design
- **obs_LPV_pol_poleregion_hinf.m**  
  Computes an LPV polytopic observer with H∞ and pole placement constraints (defined as an LMI region).

---

### 💡 Notes
- The provided functions are useful for stability analysis, observer design, and control design in LPV systems.
- The repository is intended as a companion to the book, offering practical implementations for readers.

For more details, refer to the book and the official website. 🚀

