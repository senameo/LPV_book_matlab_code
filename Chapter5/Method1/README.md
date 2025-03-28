# Example - Method 1: an $H_\infty/gH_2$ observer for suspension state estimation

This directory contains the method detailed in **Section 5.5  of Chapter 5**. It focuses on:

### 📌 Case: Force estimation using ”classical” LPV observers considering several objectives: pole placement, $gH_2$ and $H_\infty$ performance criteria.

#### 📂 Main Files
- **`Chapter5_method1.m`**  
  *This script includes the modeling definition and observer design solution.*

- **`Init_simulations_chapter5_method1.m`**  
  *This function allows running three simulation scenarios: two shown in Section 5.5 of Chapter 5 and an additional one (that includes a two-states SkyHook control for the suspension control input).*

#### 🔧 Supporting Functions
- **`obs_LPVpol_pole_h2_hinf.m`**  
  *Function that computes an LPV polytopic observer with* $gH_2$ *optimal performance, with* $H_\infty$ *and pole placement constraints (defined as LMI region).*

- **`Simulations_chapter5_method1_3scenarios.slx`**  
  *Simulink file that runs the simulation.*

#### 📈 Data for simulations
 **`road_simulation.mat; time_simulation.mat`**  
  *These data contains a Type C road profile (ISO 8608) signal of duration 20sec, scaled by 1/5 to fit the scaled INOVE platform. They are used in scenario 2.*

For further details, refer to the book's Chapter 5. 📖
