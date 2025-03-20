# Linear Parameter-Varying Control: Theory and Application to Automotive Systems

This repository contains examples from the book:  
**Linear Parameter-Varying Control: Theory and Application to Automotive Systems**  
Published by **John Wiley & Sons, Inc., Hoboken, New Jersey**  
**ISBN:** 9781394285952  

## Companion Website

This book is accompanied by a companion website:  
🔗 [www.wiley.com/go/sename/lpvcontrol](www.wiley.com/go/sename/lpvcontrol)  

The website includes access to this GitHub repository.

---

## Overview of Provided Functions

A certain number of generic functions can be reused for stability analysis, observer design, and control design. Below is a list of some key functions:

### 🔹 Stability Analysis

- **`LMI_ParameterDependentStability_grid.m`**  
  Checks the parameter-dependent stability of an autonomous LPV system given by:

$$\dot{x} = A(\rho)x$$

where:
- **x** is the state vector,
- **A(ρ)** is the parameter-dependent system matrix,
- **ρ** represents the varying parameters.

using a Parameter-Dependent Lyapunov Function.

---

### 🔹 Control Design

- **`lmiHinfStateFeedbackRobust_expstab.m`**  
  Computes a robust state feedback controller for a polytopic system by solving an LMI problem.

- **`lmiHinfStateFeedbackPolytope.m`**  
  YALMIP/Matlab function that computes an **LPV / $H_\infty$** state feedback controller by solving an LMI problem.

- **`lmiHinfPolytope.m`**  
  YALMIP/Matlab function that computes an **LPV / $H_\infty$** controller by solving an LMI problem.  
  Developed by **Charles Poussot-Vassal** during his PhD studies at **Grenoble/GIPSA-lab**.

---

### 🔹 Observer Design

- **`obs_LPV_pol_poleregion_hinf.m`**  
  Computes an **LPV polytopic observer** with **$H_\infty$** and **pole placement constraints** (defined as an LMI region).

---

## 📌 Notes

- These functions are written in **[Matlab](https://www.mathworks.com/products/matlab.html)** and rely on **[YALMIP](https://yalmip.github.io/)** for LMI writing. Solvers such as **[SeDuMi](https://sedumi.ie.lehigh.edu/)**, **[SDPT3](https://github.com/sqlp/sdpt3)**, and **[MOSEK](https://www.mosek.com/)** can be used for LMI solving.
- The provided implementations can be used for research and educational purposes.

---

## 🔗 References

For more details, please refer to the book and companion website.  
If you use part of this repository useful, please consider citing the book in your work.  

---


