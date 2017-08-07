
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

I choose N (timestep length) equals to 7 and dt (elapsed duration between timesteps) equals to 0.3. I tried smaller dt like 0.05 and 0.15 and 0.2, those dt would cause oscillation. There is no need to set N > 20 because we took time calculate them all but only take one predict point. N shouldn't be too small because we need time to tell the trend of the track.


####3. Polynomial Fitting and MPC Preprocessing

When using a polynomial to fit waypoints, I first turn the way points inreference to global coordinates to car coordinates, which would simplify the calculation. And next we only use x and y of car coordinates to get a polynomial.

####4. Model Predictive Control with Latency

I consider the latency as a new moment after we get the current state. The new steer_value and throttle_value take effect after the latency, during the latency, the car runs in former state(v and delta and a).
The car moves along x axis of car coordinate, we get the x state after the latency: v*latency + delta(latency), just ignore the latter since the latency is small.
And there are no shift in y axis.
The new v is v+ a*latency and new psi is psi+delta*latency.
We pass the state after the latency as the real state to solve the predicted points.

