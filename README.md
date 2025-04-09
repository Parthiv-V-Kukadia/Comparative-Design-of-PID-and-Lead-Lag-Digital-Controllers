# Comparative-Design-of-PID-and-Lead-Lag-Digital-Controllers
## Digital Controller Design for a Second-Order System

This repository contains a MATLAB script that designs and simulates two digital controllers – a PID controller and a Lead-Lag compensator – for a given continuous-time second-order plant. The script also creates corresponding Simulink models to visualize and analyze the closed-loop system performance under both step and ramp inputs.

## Overview

The MATLAB script performs the following steps:

1.  **System Definition:** Defines the continuous-time plant transfer function and the desired sample time for the digital implementation. It then discretizes the plant using the Zero-Order Hold (ZOH) method.

    The continuous-time plant is given by:
    $$G_{continuous}(s) = \frac{1}{(s+1)s}$$

    The sample time is set to:
    $$T_s = 0.01 \text{ s}$$

    The discrete-time plant is obtained using the `c2d` function with the 'zoh' method.

2.  **Controller Designs:**
    * **PID Controller:**
        * Initial tuning is performed using `pidtune` on the discrete-time plant to achieve a balance of performance and robustness.
        * The proportional gain ($K_p$), integral gain ($K_i$), and derivative gain ($K_d$) are further adjusted to improve the system's response, particularly by increasing the integral gain to reduce steady-state error.
        * A derivative filter coefficient ($N$) is set to 100 to reduce high-frequency noise amplification.
        * The sample time of the PID controller is explicitly set.

    * **Lead-Lag Compensator:**
        * The lead compensator is designed to improve the phase margin ($PM$) and bandwidth ($\omega_c$) of the system. The desired phase margin is set to 70 degrees at a crossover frequency of 5.5 rad/s.
        * The parameters of the lead compensator ($\alpha$ and $T_{lead}$) are calculated based on the desired phase lead required at the crossover frequency. The gain $K_{lead}$ is adjusted to place the magnitude Bode plot at 0 dB at the crossover frequency.
        * The lag compensator is designed to reduce the steady-state error, specifically for a ramp input. The parameter $\beta$ and $T_{lag}$ are chosen to provide sufficient low-frequency gain without significantly affecting the phase margin.
        * The continuous-time lead-lag compensator is then discretized using Tustin's method (`'tustin'`).

3.  **Simulink Model Generation:** The script automatically creates and populates four Simulink models:
    * `Digital_PID_System.slx`: Closed-loop system with the PID controller and a step input.
    * `Digital_LeadLag_System.slx`: Closed-loop system with the Lead-Lag compensator and a step input.
    * `Digital_PID_System_Ramp.slx`: Closed-loop system with the PID controller and a ramp input.
    * `Digital_LeadLag_System_Ramp.slx`: Closed-loop system with the Lead-Lag compensator and a ramp input.

    Each model includes:
    * A Reference input (Step or Ramp).
    * A Sum block for the error signal.
    * The designed digital controller (Discrete PID Controller or Discrete Transfer Fcn for Lead-Lag).
    * A Saturation block to limit the control output between -5 and 5.
    * The discrete-time Plant (Discrete Transfer Fcn).
    * A Quantizer block to simulate the effect of an encoder with a resolution of 1000 pulses per revolution.
    * A Scope to visualize the system output.
    * To Workspace blocks to save the output (`y_out`) and control input (`u_out`) as `timeseries` objects for analysis. For the ramp input scenarios, the reference signal is also saved (`reference_out`).

4.  **Simulation and Performance Evaluation:**
    * Simulations are run for both step and ramp inputs for a duration of 10 seconds.
    * For the ramp input simulations, the steady-state ramp error is calculated by taking the mean absolute error between the reference and the output during the last 5 seconds of the simulation.
    * For the step input simulations, step response characteristics such as overshoot, settling time, rise time, peak time, and peak value are calculated using the `stepinfo` function.
    * The Phase Margin (PM) and Gain Margin (GM) of the open-loop systems with each controller are calculated using the `margin` function.

5.  **Plotting:**
    * The step responses of the closed-loop systems with both PID and Lead-Lag controllers are plotted on the same figure for comparison.
    * Bode plots of the open-loop transfer functions with each controller are generated, displaying the Gain and Phase Margins.

6.  **Performance Metrics Display:** A table summarizing the key performance metrics for both the PID and Lead-Lag controllers is displayed in the MATLAB command window.

## Files Included

* `digital_controller_design.m`: The main MATLAB script that performs the controller design, Simulink model generation, simulation, and analysis.
* `Digital_PID_System.slx`: Simulink model for the PID controller with a step input.
* `Digital_LeadLag_System.slx`: Simulink model for the Lead-Lag controller with a step input.
* `Digital_PID_System_Ramp.slx`: Simulink model for the PID controller with a ramp input.
* `Digital_LeadLag_System_Ramp.slx`: Simulink model for the Lead-Lag controller with a ramp input.

## How to Use

1.  **Save the MATLAB script:** Download the `digital_controller_design.m` file and save it in a directory of your choice.
2.  **Open MATLAB:** Launch MATLAB.
3.  **Navigate to the directory:** In the MATLAB command window, navigate to the directory where you saved the script.
4.  **Run the script:** Execute the script by typing `digital_controller_design` in the command window and pressing Enter.

The script will:

* Design the PID and Lead-Lag controllers.
* Create and save the four Simulink models.
* Run simulations for both step and ramp inputs for each controller.
* Generate plots of the system responses and Bode diagrams.
* Display a table of performance metrics in the command window.

You can then open and further explore the generated Simulink models (`.slx` files) in the Simulink environment. The simulation data will also be available in the MATLAB workspace as `timeseries` objects (`y_pid_data`, `u_pid_data`, `y_ll_data`, `u_ll_data`, `y_pid_ramp_data`, `u_pid_ramp_data`, `ref_pid_ramp_data`, `y_ll_ramp_data`, `u_ll_ramp_data`, `ref_ll_ramp_data`).

## Performance Metrics

The script outputs the following performance metrics for both the PID and Lead-Lag controllers:

## Performance Metrics

The following table summarizes the performance metrics obtained for both the PID and Lead-Lag controllers:

| Metrics                   | PID Controller | Lead-Lag Compensator |
|---------------------------|----------------|----------------------|
| Overshoot (%)             | 1.3444         | 1.3474               |
| Ramp Error                | 0.0077447      | 0.2672               |
| Settling Time (s)         | 0.92109        | 2.0997               |
| Rise Time (s)             | 0.56622        | 1.3899               |
| Peak Time (s)             | 1.19           | 2.97                 |
| Peak Value                | 1.0284         | 1.0406               |
| Phase Margin (deg)        | 68.324         | 72.486               |
| Gain Margin (dB)          | 26.37          | 40.752               |

These metrics provide a quantitative comparison of the performance achieved by the two different controller designs for the given system. Analyze these values to understand the trade-offs between the PID and Lead-Lag controllers in terms of transient response (overshoot, settling time, rise time, peak time), steady-state error for a ramp input, and frequency domain stability margins (phase margin, gain margin).
