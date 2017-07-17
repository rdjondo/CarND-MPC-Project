# CarND-Controls-MPC
Model Predictive Control project - Self-Driving Car Engineer Nanodegree Program

In this project, I used a MPC controller to control a simulated vehicle's steering and throttle around a racing track.

---

# Video of the controller in action
https://youtu.be/9K10vUWuE9Q

# The Model

An MPC Controller incorporates the model of the system its controlling in order to optimize the input feed to the system that would optimally statisfy a control control objective and some constraints.
The incorporated model executes a number N of simulation steps and gathers some information of the future state of the system given an optimal set of imputs.
Compared to PID Controllers, MPC controllers are particulaly useful for Non-linear systems and long-medium term varying processes or processes with large inertia.

The model I used is directly derived from the simplified bicycle model presented in the video lecture. 

This is a screenshot of the equations presented.

![Simplified bicycle model equations (image)](MPC_equations.png?raw=true "Simplified bicycle model equations")

## Constraints equations

In the word of Model Predictive Control these equations 'translate' into equality constraints equations.
They will tell the optimiser to try to keep the right side of the equality roughly equal to the left in respect to a small tolerance. 

The state of the vehicle is described with its position (x, y), its speed (v), its yaw (psi), steering angle (delta), vehicle to center of track error (cte), yaw error (epsi).



    
    fg[1 + x_start + t] = x1 - (x0 + v0 * cos(psi0) * MPC::dt);
    fg[1 + y_start + t] = y1 - (y0 + v0 * sin(psi0) * MPC::dt);
    fg[1 + psi_start + t] = psi1 - (psi0 + v0/Lf * delta * dt);
    fg[1 + v_start + t] = v1 - (v0 + acc * dt);
    fg[1 + cte_start + t] = cte1 - (f_y1  - y1); // f is objective trajectory (y as a function of x)
    fg[1 + epsi_start + t] = epsi1 - (atan(Df_y) - psi1);
    

The optimiser is set with the previously mentionned constraints expressions fg[1 + x_start + t] to fg[1 + x_start + ..] so that they are 0.

The index t helps us iterate through the different temporal values of the model predictor, from t=0 to t=N*dt.


## Cost function

The optimizer attempts to minimize the cost function whilst respect the problem constraints.
The cost function is the sum of various individual costs I will detail below.

    
    // Costs the distance to the center of the racing track
    fg[0] += pow(vars[cte_start + t], 2);  
    
    // Costs the difference in vehicle heading from the ideal trajectory (center of racing track)
    fg[0] += pow(vars[epsi_start + t], 2); 
    
    // Very high cost for the target speed. (High regularization constant to compensate for the lateral acceleration cost below)
    fg[0] += 100*pow(vars[v_start + t] - ref_v , 2); 
    
    // Cost the use of steering.
    fg[0] += CppAD::pow(vars[a_start + t], 2);
    
    // Cost the lateral acceleration, the higher the speed and the turn angle together, the higher the cost
    fg[0] += 1.8e-4*pow(vars[v_start + t]*vars[v_start + t]/(cos(2*vars[delta_start + t])+1e-6), 2);
    
    
    // Keep the continuity between sequential optimizations.
    fg[0] += 10*pow(acc_last_value - vars[a_start], 2);
    fg[0] += 100*pow(delta_last_value - vars[delta_start], 2);
    
    // Minimizes the gap between two consecutive steps within the same simulation
    for (size_t t = 0; t < MPC::N - 2; ++t) {
      fg[0] += pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    


# Timestep Length and Elapsed Duration (N & dt)

I chose N (prediction horizon length) to be 20 samples and dt (elapsed duration between timesteps) to be 0.05 ms.
This gives a prediction horizon of 1 second.
n=0.05 seems to give a good enough approximation (discretization) of the vehicle kinematics.
N=20 was chosen to get a solver time that would be fast enough(below 25 ms on my machine). 25 ms as a control rate has shown to be fast enough in the simulation given the MPC problem setup.


# Polynomial Fitting and MPC Preprocessing

The simulator supplies waypoints to the optimiser.
Those waypoints are a list of (x,y) position in the map coordinates.

In order to simplify the optimization problem, I considered that the vehicle would be assumed to be located at position (0, 0) with a 0.0 rad heading.
I assumed that in most cases the car is pretty much heading in the right direction. In addition we assume that the fitted function is going to be bijective (there is a single x for a single y). This considerations make it possible to fit a small degree polynomial around the waypoints ahead of the car.

The polynomial fitting function was provided by udacity and is a port of the Julia language polyfit function. 

# Model Predictive Control with Latency

We know that the incorporated model executes a number N of simulation steps and gathers some information of the future state of the system given an optimal set of imputs. If the optimal future inputs are stable enough or made constant during the latency, then we can use the simulated solution outputs from the MPC controller to predict what a future input should be.

I chose to add in the cost function a cost element that would penalize turns at high speeds : V^2/cos(delta). It turns out this cost has the effect of calculating as a solution a convex trajectory accross the prediction window. This is great because it makes the steering feel very stable.

With this I let the MPC run and I simply chose for the actuator input the optimum steering angle delayed by 100 ms (the latency).

To make the car drive faster on the race track, I added to the control loop a variable speed reference.
This speed target (110kph) is reduced to (60 kph) changes when the car predicts a turn.


# Future work
  - Experiment with hard constraints on the center to track error.
  - Experiment with hard constraints on lateral acceleration of the vehicle (V^2/R) 

# Caveats

Controllers implemented for this project are reviewed for robustness issues.

There are a number of hazards that could arise with the current implementation. 

* MPC Solver convergence issues - no appropriate solution was found
  - We don't have a redundant system that could take over in the case of controller failure

* Steady state errors due to process noise (ie severe cross wind, steering/throttle faults, telemetry and sensor noise)
  -  The MPC controller is blindy taking the telemetry and sensor data without taking in account the uncertainty of the data
  - The Controller is not monitoring the quality of the data it receives
  - The controller cannot compensate for steady state errors due to sensor errors, vehicle mechanical issues or changes on tire friction (grip).

* Incomplete map knowledge
  - We don't know the boundary of the drivable surface, so the controller cannot account for it for ensuring that the car is controller within boundaries of the road.


As we can see this MPC controller as implemented is far from being production-ready and there would be a large amount of work involved to make it production ready. 
