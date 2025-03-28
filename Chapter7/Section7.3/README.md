# Example - LPV / $H_\infty$ Control with real-time varying closed-loop performances (using a polytopic approach)

This directory contains Matlab and Simulink files to run the example provided in **Section 7.3 of Chapter 7**. The files to be considered are:

### 📂 Main File
- **`Section73.m`**  
  *Contains the whole program.*

#### 🔧 Supporting Functions
- **`InputFilter4Polytopic.m`**  
  *Matlab function to apply an input filtering method, needed to consider the polytopic approach.*

- **`lmiHinfPolytope.m`**  
 *Yalmip/Matlab function that computes the LPV /* $H_\infty$ *controller by solving the LMI problem.*

- **`Simulations_section73.slx`**  
  *Simulink function that executes the simulation scenarios (the scenarios are chosen in the main file).*

For further details, refer to the book's Chapter 7. 📖
