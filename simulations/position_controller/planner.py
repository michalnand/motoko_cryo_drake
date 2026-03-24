import numpy


class Planner:
    def __init__(self, acc_fwd, dec_fwd, acc_w_max, mpc_horizon, dt):
        """
        dt          : time step [s], e.g. 1/250
        acc_fwd     : forward acceleration limit [m/s²]  (positive)
        dec_fwd     : forward deceleration limit [m/s²]  (positive, typically larger)
        acc_w_max   : angular acceleration limit [rad/s²] (symmetric)
        mpc_horizon : prediction horizon for circle motion
        """
        
        self.acc_fwd = acc_fwd
        self.dec_fwd = dec_fwd
        self.acc_w_max = acc_w_max
        self.mpc_horizon = mpc_horizon
        self.dt = dt

    def set_position(self, state, x_req, a_req):
        """
        Compute next-step position and angle setpoints with asymmetric acceleration limits.

        Parameters
        ----------
        state : array-like of 4 floats
            [x_dist, x_vel, x_theta, x_omega]
        x_req : float
            target distance [m]
        a_req : float
            target angle [rad]

        Returns
        -------
        (x_new, a_new) : tuple of float
        """
        dt = self.dt
        x, v, a, w = state

        # --- forward axis ---
        v_req = (x_req - x) / dt
        acc_req = numpy.clip((v_req - v) / dt, -self.dec_fwd, self.acc_fwd)
        v_cmd = v + acc_req * dt
        x_new = x + v_cmd * dt

        # --- angular axis ---
        w_req = (a_req - a) / dt
        alpha_req = numpy.clip((w_req - w) / dt, -self.acc_w_max, self.acc_w_max)
        w_cmd = w + alpha_req * dt
        a_new = a + w_cmd * dt

        return x_new, a_new

    def set_circle_motion(self, state, r_req, v_req):
        """
        Generate MPC reference trajectory for circular arc motion.

        Parameters
        ----------
        state : array-like of 4 floats
            [x_dist, x_vel, x_theta, x_omega]
        r_req : float
            turning radius [m], must be > 0
        v_req : float
            target forward velocity [m/s]

        Returns
        -------
        trajectory : numpy.ndarray of shape (mpc_horizon, 2)
            columns: [x_position, angle]
        """
        dt = self.dt
        x, v, a, w = state

        # required acceleration, clamped asymmetrically
        acc_req = numpy.clip((v_req - v) / dt, -self.dec_fwd, self.acc_fwd)

        v_curr = v
        pred_x = x  
        pred_a = a

        trajectory = numpy.zeros((self.mpc_horizon, 2))

        for n in range(self.mpc_horizon):
            v_curr += acc_req * dt
            v_curr = numpy.clip(v_curr, -numpy.abs(v_req), numpy.abs(v_req))

            pred_x += v_curr * dt
            pred_a += (v_curr / r_req) * dt

            trajectory[n, 0] = pred_x
            trajectory[n, 1] = pred_a

        return trajectory