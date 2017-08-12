
###Implementation

####1.The Model

There are 6 items consist of the state: x, y, psi, v, cte, epsi;
2 items consist of actuators: delta(steer_angle) and a(throttle_value).
the update equations are:

    x1 = (x0 + v0 * cos(psi0) * dt);
    y1 = (y0 + v0 * sin(psi0) * dt);
    psi1 = (psi0 + v0 * delta0 / Lf * dt);
    v1 = (v0 + a0 * dt);
    cte1 = ((f0 - y0) + (v0 * sin(epsi0) * dt));
    epsi1 = ((psi0 - psides0) + v0 * delta0 / Lf * dt);  

####2. Timestep Length and Elapsed Duration (N & dt)

I choose N (timestep length) equals to 15 and dt (elapsed duration between timesteps) equals to 0.08. Because the shorter dt is, the less error will be when the curve approximated as line, the more accurate MPC controller will be. Set N bigger to tell the trend of the track.


####3. Polynomial Fitting and MPC Preprocessing

When using a polynomial to fit waypoints, I first turn the way points inreference to global coordinates to car coordinates, which would simplify the calculation. And next we only use x and y of car coordinates to get a polynomial.

####4. Model Predictive Control with Latency

I consider the latency as a middle state of the MPC control result. The start point's status influenced by current steer_value and throttle_value. And we calculate the second point, when the latency time elapsed and the new steer angle and throttle really take effects.
Set delta and a of the first point equals to current steer and throttle value. 
    vars_lowerbound[delta_start]=prevDelta;
    vars_upperbound[delta_start]=prevDelta;
    vars_lowerbound[a_start]=prevA;
    vars_upperbound[a_start]=prevA;
Take the second delta and a as control input.
    vector<double> predict;
    predict.push_back(solution.x[delta_start]);
    predict.push_back(solution.x[a_start+1]);
Save control input delta and a as init status of next iteration.
    prevDelta=solution.x[delta_start+1];
    prevA=solution.x[a_start+1];