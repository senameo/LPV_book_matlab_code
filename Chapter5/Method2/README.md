# Example - Method 2: a $H_\infty$ filtering approach for damper force estimation

This directory contains the method detailed in **Section 5.6  of Chapter 5**. It focuses on:

### 📌 Case: Force estimation using LPV filters, designed in an $H_\infty$ framework (as a dual problem to control design).

#### 📂 Main Files
- **`Chapter5_method2.m`**  
  *This script includes the modeling definition and LPV filter design solution.*

- **`Init_simulations_chapter5_method2.m`**  
  *This function allows running three simulation scenarios: two shown in Section 5.6 of Chapter 5 and an additional one (that includes a two-states SkyHook control for the suspension control input).*

#### 🔧 Supporting Functions
- **`lmiHinfPolytope.m`**  
  *Yalmip/Matlab function that computes the LPV /* $H_\infty$ *controller by solving the LMI problem. It is used here in a way to design an LPV polytopic filter.*

- **`Pgenerationsimulink_chapter5_method2.slx`**  
  *Simulink function that allows to define the generalized plant to be considered to design the LPV polytopic filter.*

- **`Simulations_chapter5_method2.slx`**  
  *Simulink file that runs the simulation.*

#### 📈 Data for simulations
 **`road_simulation.mat; time_simulation.mat`**  
  *These data contains a Type C road profile (ISO 8608) signal of duration 20sec, scaled by 1/5 to fit the scaled INOVE platform. They are used in scenario 2.*

For further details, refer to the book's Chapter 5. 📖
