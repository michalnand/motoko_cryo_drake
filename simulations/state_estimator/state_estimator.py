import numpy
    

class StateEstimator:
    """
    Hybrid state estimator for robot distance and angle measurements.

    - **Positions** (distance, theta) are denoised with a first-order
      Exponential Moving Average (EMA) whose bandwidth is set via a
      cutoff frequency parameter.
    - **Velocities** (linear, angular) are estimated with a causal
      Savitzky-Golay derivative filter for better noise rejection.
    """

    def __init__(self, order=3, window_size=11, dt=0.01, cutoff_freq=10.0):
        """
        Parameters
        ----------
        order : int
            Polynomial order for the Savitzky-Golay velocity filter.
        window_size : int
            Number of samples in the SG sliding window (must be odd).
        dt : float
            Sampling period in seconds.
        cutoff_freq : float
            EMA cutoff frequency in Hz for position smoothing.
            Higher  → less smoothing, faster response.
            Lower   → more smoothing, slower response.
        """
        assert window_size % 2 == 1, "window_size must be odd"
        assert order < window_size, "order must be less than window_size"
        assert cutoff_freq > 0, "cutoff_freq must be positive"

        self.order = order
        self.window_size = window_size
        self.dt = dt
        self.cutoff_freq = cutoff_freq

        # ── EMA coefficient from cutoff frequency ──
        # Derived from the continuous RC low-pass:  alpha = dt / (RC + dt)
        # where RC = 1 / (2 * pi * f_c)
        rc = 1.0 / (2.0 * numpy.pi * cutoff_freq)
        self.alpha = dt / (rc + dt)

        # ── Savitzky-Golay velocity coefficients ──
        self.buf_dist  = numpy.zeros(window_size)
        self.buf_theta = numpy.zeros(window_size)
        self.buf_count = 0

        self._coeff_deriv = self._compute_sg_derivative_coefficients()

        # ── outputs ──
        self.x_dist_est  = 0.0
        self.x_vel_est   = 0.0
        self.x_theta_est = 0.0
        self.x_omega_est = 0.0

        print(self._coeff_deriv)

    def _compute_sg_derivative_coefficients(self):
        """
        Precompute causal Savitzky-Golay coefficients for the 1st derivative.

        Window indices run from -(N-1) to 0 (right-aligned / causal),
        scaled by dt so the dot product directly yields physical velocity.
        """
        N = self.window_size
        m = self.order

        t = numpy.arange(-(N - 1), 1, dtype=numpy.float64) * self.dt

        # Vandermonde matrix  [1, t, t^2, ..., t^m]
        A = numpy.vander(t, m + 1, increasing=True)  # shape (N, m+1)

        # pseudo-inverse → row 1 gives 1st-derivative coefficients
        pinv = numpy.linalg.pinv(A)  # shape (m+1, N)
        coeff_deriv = pinv[1, :]     # shape (N,)

        return coeff_deriv

    def step(self, x_dist, x_theta):
        """
        Push new measurements and return filtered estimates.

        Returns
        -------
        x_dist_est  : float  – EMA-smoothed distance
        x_theta_est : float  – EMA-smoothed angle
        x_vel_est   : float  – SG-estimated linear velocity
        x_omega_est : float  – SG-estimated angular velocity
        """
        N = self.window_size

        # ── EMA for positions ──
        self.x_dist_est  += self.alpha * (x_dist  - self.x_dist_est)
        self.x_theta_est += self.alpha * (x_theta - self.x_theta_est)

        # ── SG buffer update ──
        self.buf_dist[:-1]  = self.buf_dist[1:]
        self.buf_dist[-1]   = self.x_dist_est

        self.buf_theta[:-1] = self.buf_theta[1:]
        self.buf_theta[-1]  = self.x_theta_est

        self.buf_count = min(self.buf_count + 1, N)

        if self.buf_count < N:
            # not enough samples for SG → zero velocity
            self.x_vel_est   = 0.0
            self.x_omega_est = 0.0
        else:
            # velocity estimation via SG 1st derivative
            self.x_vel_est   = numpy.dot(self._coeff_deriv, self.buf_dist)
            self.x_omega_est = numpy.dot(self._coeff_deriv, self.buf_theta)

        return self.x_dist_est, self.x_theta_est, self.x_vel_est, self.x_omega_est