# Example - Nonlinear System Control

This directory contains the example detailed in **Section 4.2 of Chapter 4**. It focuses on:

### 📌 Case: Control of a Nonlinear System Using an H∞ / LPV Control Approach, Through a Polytopic Formulation

#### 📂 Main Files
- **`Section42.m`**  
  *This script includes the modeling definition, control formulation, and control solution.*

- **`Init_simulations_section42.m`**  
  *This function allows running both simulation scenarios shown in Section 4.2 of Chapter 4.*

#### 🔧 Supporting Function
- **`lmiHinfPolytope.m`**  
  *Yalmip/Matlab function that computes the LPV /* $H_\infty$ *controller by solving the LMI problem.*

- **`Simulations_section42.slx`**  
  *Simulink file that runs the simulation.*

#### ⚠️ Important Notes:
- While not detailed in the text, the simulation scenario includes an input disturbance of magnitude 1, which acts:
  - At time **t = 15 sec** for Case 1.
  - At time **t = 20 sec** for Case 2.

For further details, refer to the book's Chapter 4. 📖
