# Example - Method 3: a nonlinear parameter varying approach for state estimation

This directory contains the method detailed in **Section 5.7  of Chapter 5**. It focuses on:

### 📌 Case: Force estimation using NLPV observers, designed in an $H_\infty$ framework using either the polytopic approach or the grid-based one.

#### 📂 Main Files
- **`Chapter5_method3.m`**  
  *This script includes the modeling definition and LPV filter design solution.*

- **`Init_simulations_chapter5_method3.m`**  
  *This function allows running three simulation scenarios: two shown in Section 5.7 of Chapter 5 and an additional one (that includes a two-states SkyHook control for the suspension control input).*

#### 🔧 Supporting Functions
- **`LMI_NLPV_Polytopic.m`**  
  *Yalmip/Matlab function that computes the NLPV /* $H_\infty$ * (polytopic) observer by solving the LMI problem for poplytopic systems*

- **`LMI_NLPV_Gridding.m`**  
  *Yalmip/Matlab function that computes the NLPV /* $H_\infty$ * observer by solving the LMI problem using a grid-based method.*

- **`Simulations_chapter5_method3.slx`**  
  *Simulink file that runs the simulation.*

#### 📈 Data for simulations
 **`road_simulation.mat; time_simulation.mat`**  
  *These data contains a Type C road profile (ISO 8608) signal of duration 20sec, scaled by 1/5 to fit the scaled INOVE platform. They are used in scenario 2.*

For further details, refer to the book's Chapter 5. 📖