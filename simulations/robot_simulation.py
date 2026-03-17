import numpy
import AILibs


class ShaperVelAcc:
    def __init__(self, vn_max, vp_max, an_max, ap_max, alpha, dt):
        self.vn_max = float(vn_max)
        self.vp_max = float(vp_max)
        self.an_max = float(an_max)
        self.ap_max = float(ap_max)
        self.alpha = float(alpha)
        self.dt = float(dt)

        self.x = 0.0
        self.v = 0.0
        self.a = 0.0
        self.x_fil = 0.0

    def reset(self):
        self.x = 0.0
        self.v = 0.0
        self.a = 0.0
        self.x_fil = 0.0

    def step(self, x_target):
        dx = x_target - self.x

        if dx > 0:
            v_brake_pos = numpy.sqrt(2.0 * abs(self.an_max) * dx)
            v_brake_neg = self.vn_max
        elif dx < 0:
            v_brake_pos = self.vp_max
            v_brake_neg = -numpy.sqrt(2.0 * self.ap_max * abs(dx))
        else:
            v_brake_pos = 0.0
            v_brake_neg = 0.0

        v_upper = min(self.vp_max, v_brake_pos)
        v_lower = max(self.vn_max, v_brake_neg)

        v_req = dx / self.dt
        v_req = numpy.clip(v_req, v_lower, v_upper)

        a_req = (v_req - self.v) / self.dt
        self.a = numpy.clip(a_req, self.an_max, self.ap_max)

        self.v += self.a * self.dt
        self.v = numpy.clip(self.v, v_lower, v_upper)

        self.x += self.v * self.dt
        self.x_fil = (1.0 - self.alpha) * self.x_fil + self.alpha * self.x

        return self.x_fil


class RobotSimulation:
    """
    Full robot simulation pipeline:
        target (distance, angle) -> shaper -> LQR -> discrete dynamics

    State vector: [distance, distance_dot, theta, theta_dot]
    Control vector: [u_forward, u_turn]

    Cartesian pose (px, py, heading) is integrated from the robot state
    for rendering purposes.
    """

    def __init__(self, A, B, Q, R, dt,
                 vn_max_dist, vp_max_dist, an_max_dist, ap_max_dist, alpha_dist,
                 vn_max_angle, vp_max_angle, an_max_angle, ap_max_angle, alpha_angle):
        """
        Parameters
        ----------
        A : array (4,4) - discrete state matrix
        B : array (4,2) - discrete input matrix
        Q : array (4,4) - LQR state cost
        R : array (2,2) - LQR input cost
        dt : float       - sampling period
        vn_max_dist ... alpha_dist : shaper params for distance channel
        vn_max_angle ... alpha_angle : shaper params for angle channel
        """
        self.A = numpy.array(A, dtype=numpy.float64)
        self.B = numpy.array(B, dtype=numpy.float64)
        self.Q = numpy.array(Q, dtype=numpy.float64)
        self.R = numpy.array(R, dtype=numpy.float64)
        self.dt = float(dt)

        # synthesize LQR with integral action
        self.controller = AILibs.LQRIDiscrete(self.A, self.B, self.Q, self.R)

        # create discrete dynamical system
        self.dynamics = AILibs.DynamicalSystemDiscrete(self.A, self.B, None)

        # create shapers for distance and angle channels
        self.shaper_dist = ShaperVelAcc(
            vn_max_dist, vp_max_dist, an_max_dist, ap_max_dist, alpha_dist, dt
        )
        self.shaper_angle = ShaperVelAcc(
            vn_max_angle, vp_max_angle, an_max_angle, ap_max_angle, alpha_angle, dt
        )

        # internal state
        self.x = None
        self.integral_action = None

        # cartesian pose for rendering: (px, py, heading)
        self.px = 0.0
        self.py = 0.0
        self.heading = 0.0

        self.reset()

    def reset(self):
        """Reset all internal state to zero."""
        self.x = self.dynamics.reset()
        self.integral_action = numpy.zeros((self.B.shape[1], 1))

        self.shaper_dist.reset()
        self.shaper_angle.reset()

        self.px = 0.0
        self.py = 0.0
        self.heading = 0.0

        return self.get_state()

    def forward(self, target_distance, target_angle):
        """
        Run one time-step of the full pipeline.

        Parameters
        ----------
        target_distance : float - desired forward distance
        target_angle    : float - desired heading angle [rad]

        Returns
        -------
        state : dict with keys:
            'x'       : (4,1) ndarray - full state [d, d_dot, theta, theta_dot]
            'u'       : (2,1) ndarray - control input [u_fwd, u_turn]
            'xr'      : (4,1) ndarray - shaped reference state
            'px'      : float - cartesian x position
            'py'      : float - cartesian y position
            'heading' : float - cartesian heading [rad]
        """
        # shape the reference signals
        dist_shaped = self.shaper_dist.step(target_distance)
        angle_shaped = self.shaper_angle.step(target_angle)

        # build reference state vector (velocities desired = 0)
        xr = numpy.array([[dist_shaped], [0.0], [angle_shaped], [0.0]])

        # compute LQR control
        u, self.integral_action = self.controller.forward(
            xr, self.x, self.integral_action
        )

        # step dynamics
        self.x, _ = self.dynamics.forward_state(u)

        # integrate cartesian pose for rendering
        # distance_dot = x[1], theta = x[2]
        velocity = float(self.x[1, 0])
        theta = float(self.x[2, 0])

        self.heading = theta
        self.px += velocity * numpy.cos(self.heading) * self.dt
        self.py += velocity * numpy.sin(self.heading) * self.dt

        return {
            'x': self.x.copy(),
            'u': u.copy(),
            'xr': xr.copy(),
            'px': self.px,
            'py': self.py,
            'heading': self.heading,
        }

    def get_state(self):
        """Return current state without stepping."""
        return {
            'x': self.x.copy() if self.x is not None else None,
            'u': None,
            'xr': None,
            'px': self.px,
            'py': self.py,
            'heading': self.heading,
        }
