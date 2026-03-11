import numpy


class StateEstimator:
    """
    Savitzky-Golay filter for denoising robot distance and angle measurements
    and estimating their velocities.

    Uses a polynomial fit over a sliding window of past samples.
    The smoothed value is the polynomial evaluated at the newest sample,
    and the velocity is the analytical derivative evaluated at the same point.
    """

    def __init__(self, order=3, window_size=11, dt=0.01):
        assert window_size % 2 == 1, "window_size must be odd"
        assert order < window_size, "order must be less than window_size"

        self.order = order
        self.window_size = window_size
        self.dt = dt

        # ring buffers for distance and theta
        self.buf_dist  = numpy.zeros(window_size)
        self.buf_theta = numpy.zeros(window_size)
        self.buf_count = 0

        # precompute Savitzky-Golay convolution coefficients
        # for smoothing (0th derivative) and 1st derivative
        self._coeff_smooth, self._coeff_deriv = self._compute_sg_coefficients()

        # outputs
        self.x_dist_est  = 0.0
        self.x_vel_est   = 0.0
        self.x_theta_est = 0.0
        self.x_omega_est = 0.0

        #self.x_dist_est_prev  = 0.0
        #self.x_theta_est_prev = 0.0

        print(f"SG coefficients (smooth): {self._coeff_smooth}")
        print(f"SG coefficients (deriv): {self._coeff_deriv}")

    def _compute_sg_coefficients(self):
        """
        Precompute the Savitzky-Golay convolution coefficients for
        0th derivative (smoothing) and 1st derivative.

        The window indices go from -(N-1) to 0 (causal / right-aligned),
        so that the filtered value corresponds to the most recent sample.
        """
        N = self.window_size
        m = self.order

        # causal window: indices -N+1, -N+2, ..., -1, 0
        # scaled by dt so derivative coefficients come out in physical units
        t = numpy.arange(-(N - 1), 1, dtype=numpy.float64) * self.dt

        # Vandermonde matrix  [1, t, t^2, ..., t^m]
        A = numpy.vander(t, m + 1, increasing=True)  # shape (N, m+1)

        # least-squares pseudo-inverse
        # pinv = (A^T A)^{-1} A^T
        pinv = numpy.linalg.pinv(A)  # shape (m+1, N)

        # smoothed value  = 0th row of pinv dotted with the buffer
        coeff_smooth = pinv[0, :]  # shape (N,)

        # 1st derivative  = 1st row of pinv dotted with the buffer
        coeff_deriv = pinv[1, :]   # shape (N,)

        return coeff_smooth, coeff_deriv

    def step(self, x_dist, x_theta):
        """
        Push new distance and theta measurements, return denoised values
        and velocity / angular-velocity estimates.
        """
        N = self.window_size

        # shift buffers left and insert the newest sample at the end
        self.buf_dist[:-1]  = self.buf_dist[1:]
        self.buf_dist[-1]   = x_dist

        self.buf_theta[:-1] = self.buf_theta[1:]
        self.buf_theta[-1]  = x_theta

        self.buf_count = min(self.buf_count + 1, N)

        if self.buf_count < N:
            # not enough samples yet – pass through raw values, zero velocity
            self.x_dist_est  = x_dist
            self.x_theta_est = x_theta
            self.x_vel_est   = 0.0
            self.x_omega_est = 0.0
        else:
            # smoothed position (0th derivative)
            self.x_dist_est  = numpy.dot(self._coeff_smooth, self.buf_dist)
            self.x_theta_est = numpy.dot(self._coeff_smooth, self.buf_theta)

            # velocity estimation (1st derivative)
            self.x_vel_est   = numpy.dot(self._coeff_deriv, self.buf_dist)
            self.x_omega_est = numpy.dot(self._coeff_deriv, self.buf_theta)

            #self.x_vel_est   = (self.x_dist_est  - self.x_dist_est_prev)/self.dt
            #self.x_omega_est = (self.x_theta_est - self.x_theta_est_prev)/self.dt

            #self.x_dist_est_prev  = self.x_dist_est
            #self.x_theta_est_prev = self.x_theta_est

        return self.x_dist_est, self.x_theta_est, self.x_vel_est, self.x_omega_est