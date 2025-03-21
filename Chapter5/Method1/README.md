# Example - Method 1: an $H_\infty/gH_2$ observer for suspension state estimation

This directory contains the method detailed in **Section 5.5  of Chapter 5**. It focuses on:

### 📌 Case: ”classical” LPV observers are designed considering several objectives: pole placement, $gH_2$ and $H_\infty$ performance criteria.

#### 📂 Main File
- **`Chapter5_method1.m`**  
  *This script includes the modeling definition and observer design solution.*

#### 🔧 Supporting Functions
- **`obs_LPVpol_pole_h2_hinf.m`**  
  *Function that computes an LPV polytopic observer with $gH_2$ optimal performance, with $H_\infty$ and pole placement constraints (defined as LMI region).*

  
- **`Init_simulations_chapter5_method1.m`**  
  *This function allows running three simulation scenarios: two shown in Section 5.5 of Chapter 5 and one additional one.*


- **`Simulations_chapter5_method1_3scenarios.slx`**  
  *Simulink file that runs the simulation.*

For further details, refer to the book's Chapter 5. 📖
