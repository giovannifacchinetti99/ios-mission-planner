from enum import IntEnum

import numpy as np

from ios_mission_planner.dynamics.attitude.determination import triad_attitude
from ios_mission_planner.dynamics.relative.cw import cw_targeting
from ios_mission_planner.gnc.control import ApproachMPC, HoldController, tracking_weights
from ios_mission_planner.gnc.guidance import GlideslopeReference, plan_transfer
from ios_mission_planner.sim.docking import approach_basis


class Phase(IntEnum):
    """Phases of the approach, in the order in which they are flown."""

    HOLD_FAR = 0
    TRANSFER = 1
    HOLD_MID = 2
    APPROACH = 3
    HOLD_NEAR = 4
    FINAL_APPROACH = 5
    DOCKED = 6
    ABORT = 7


PHASE_LABELS = {
    Phase.HOLD_FAR: "hold at S1",
    Phase.TRANSFER: "transfer S1 to S2",
    Phase.HOLD_MID: "hold at S2",
    Phase.APPROACH: "forced approach S2 to S3",
    Phase.HOLD_NEAR: "hold at S3",
    Phase.FINAL_APPROACH: "final approach",
    Phase.DOCKED: "docked",
    Phase.ABORT: "abort",
}

_HOLD_PHASES = (Phase.HOLD_FAR, Phase.HOLD_MID, Phase.HOLD_NEAR)
_MONITORED_PHASES = (Phase.APPROACH, Phase.HOLD_NEAR, Phase.FINAL_APPROACH)


class MissionManager:
    """
    Sequencer of the approach: it decides which phase the deputy is in,
    produces the translational acceleration command of that phase with
    the guidance law and the controller that belong to it, tells the
    attitude guidance what to point at, and watches the conditions that
    call for a collision avoidance maneuver.

    The profile is the classical one for an approach along the port axis
    of a cooperative target. The deputy waits at a far hold point S1
    while its navigation converges; it transfers to a hold point S2 with
    two impulsive burns and mid-course corrections; it flies a forced
    straight-line approach to a near hold point S3, with a closing speed
    proportional to the distance; it waits there for the go; and it
    translates at constant speed until the ports touch.

    The far hold points are kept by a linear quadratic regulator. From
    the start of the forced approach on, where the corridor and the
    closing speed limits matter, the command comes from the model
    predictive controller.

    Everything is decided from the navigation estimate, never from the
    true state: the manager knows what the on-board software would know.

    Parameters
    ----------
    guidance : ios_mission_planner.sim.config.GuidanceConfig
    control : ios_mission_planner.sim.config.ControlConfig
    n : float
        Mean motion of the chief [rad/s].
    dt : float
        Period at which update is called [s].
    max_acceleration : float
        Acceleration available on each body axis [m/s^2].
    chief_port_position, chief_port_axis : array_like, shape (3,)
        Position [m] and outward normal of the chief port, in LVLH.
    deputy_port_position_body, deputy_port_axis_body, deputy_roll_axis_body : array_like
        Geometry of the deputy port, in body axes.
    start_hold : int
        Hold point at which the mission starts (0, 1 or 2).
    lighting_check : callable, optional
        Function (t, duration) returning whether the lighting is
        favourable from t for the given duration.

    Author: Giovanni Facchinetti, 2026
    Reference: Fehse, Automated Rendezvous and Docking of Spacecraft
    """

    def __init__(self, guidance, control, n, dt, max_acceleration, chief_port_position, chief_port_axis,
                 deputy_port_position_body, deputy_port_axis_body, deputy_roll_axis_body,
                 start_hold=0, lighting_check=None):
        self.guidance = guidance
        self.control = control
        self.n = n
        self.dt = dt
        self.max_acceleration = max_acceleration
        self.lighting_check = lighting_check

        axis, lateral_1, lateral_2 = approach_basis(chief_port_axis)
        self.axis, self.lateral_1, self.lateral_2 = axis, lateral_1, lateral_2

        # attitude of the deputy when the ports are aligned, and position
        # of its center of mass when they touch
        self.aligned_lvlh_to_body = triad_attitude(deputy_port_axis_body, deputy_roll_axis_body, -axis, lateral_2)
        arm = self.aligned_lvlh_to_body.T @ np.asarray(deputy_port_position_body, dtype=float)
        self.contact_position = np.asarray(chief_port_position, dtype=float) - arm

        self.hold_positions = [self.contact_position + d * axis for d in guidance.hold_distances]

        period = control.translation_period
        self.hold_controllers = []
        for distance in guidance.hold_distances[:2]:
            tolerance = max(control.hold_position_tolerance_min,
                            control.hold_position_tolerance_fraction * distance)
            velocity_tolerance = np.sqrt(control.hold_acceleration_scale * tolerance)
            q, r = tracking_weights(tolerance, velocity_tolerance, control.hold_acceleration_scale)
            self.hold_controllers.append(HoldController(n, period, q, r))

        self.constraint_matrix, self._constraint_offset = self._state_constraints()
        q, r = tracking_weights(control.mpc_position_tolerance, control.mpc_velocity_tolerance,
                                control.mpc_acceleration_scale)
        self.approach_mpc = ApproachMPC(n, period, control.mpc_horizon, q, r, max_acceleration,
                                        self.constraint_matrix)
        q, r = tracking_weights(control.final_position_tolerance, control.final_velocity_tolerance,
                                control.final_acceleration_scale)
        self.final_mpc = ApproachMPC(n, period, control.mpc_horizon, q, r, max_acceleration,
                                     self.constraint_matrix)

        self.phase = (Phase.HOLD_FAR, Phase.HOLD_MID, Phase.HOLD_NEAR)[start_hold]
        self.phase_start = 0.0
        self.pointing_mode = "port" if start_hold == 2 else "target"
        self.events = []
        self.abort_reason = None
        self.finished = False

        self._command = np.zeros(3)
        self._next_control = 0.0
        self._burn = None
        self._braking = False
        self._corrections = []
        self._arrival_time = None
        self._carrot = None
        self._abort_end = None

        # diagnostics of the predictive controller
        self.mpc_solves = 0
        self.mpc_fallbacks = 0
        self.mpc_max_slack = 0.0

    # ------------------------------------------------------------------
    # geometry
    # ------------------------------------------------------------------

    def _state_constraints(self):
        """
        Matrix C and constant part of the bound d of the constraints
        C x <= d that the predictive controller softly enforces: the four
        faces of the approach corridor, the limit on the closing speed,
        and the plane of the chief port.
        """

        g = self.guidance
        a, e1, e2 = self.axis, self.lateral_1, self.lateral_2
        c0 = self.contact_position
        slope = np.tan(np.deg2rad(g.corridor_half_angle_deg))

        rows, bounds = [], []
        for lateral in (e1, -e1, e2, -e2):
            position_row = lateral - slope * a
            rows.append(np.concatenate([position_row, np.zeros(3)]))
            bounds.append(g.corridor_radius + position_row @ c0)

        rows.append(np.concatenate([-g.closing_speed_slope * a, -a]))
        bounds.append(g.closing_speed_floor - g.closing_speed_slope * (a @ c0))

        rows.append(np.concatenate([-a, np.zeros(3)]))
        bounds.append(-(a @ c0))

        return np.array(rows), np.array(bounds)

    def axial_distance(self, position):
        """Distance still to cover along the port axis before contact [m]."""

        return float((np.asarray(position) - self.contact_position) @ self.axis)

    def lateral_distance(self, position):
        """Distance from the approach axis [m]."""

        offset = np.asarray(position) - self.contact_position
        return float(np.hypot(offset @ self.lateral_1, offset @ self.lateral_2))

    # ------------------------------------------------------------------
    # sequencing
    # ------------------------------------------------------------------

    def _enter(self, phase, t, note=""):
        self.phase = phase
        self.phase_start = t
        self._next_control = t
        self._burn = None
        self._braking = False
        self.events.append((t, phase, note))

    def notify_contact(self, t):
        """Called by the simulation when the ports touch."""

        self._enter(Phase.DOCKED, t)
        self._command = np.zeros(3)
        self.finished = True

    def update(self, t, state, position_std, measurement_age, pointing_error, applied_acceleration):
        """
        Advance the sequencer by one step and return the acceleration
        command.

        Parameters
        ----------
        t : float
            Time [s].
        state : array_like, shape (6,)
            Navigation estimate of the relative state [m, m/s].
        position_std : array_like, shape (3,)
            Standard deviation of the position estimate [m].
        measurement_age : float
            Time since the last valid relative measurement [s].
        pointing_error : float
            Estimated attitude error with respect to the command [rad].
        applied_acceleration : array_like, shape (3,)
            Acceleration that the thrusters were commanded to deliver
            during the previous step, in LVLH [m/s^2].

        Returns
        -------
        numpy.ndarray, shape (3,)
            Commanded acceleration in the LVLH frame [m/s^2].
        """

        state = np.asarray(state, dtype=float)

        if self._burn is not None:
            self._burn = self._burn - np.asarray(applied_acceleration) * self.dt

        if self.guidance.abort_enabled and self.phase in _MONITORED_PHASES:
            reason = self._abort_condition(state, position_std, measurement_age, pointing_error)
            if reason is not None:
                self._start_abort(t, reason)

        if not self.pointing_mode == "port" and self.phase != Phase.ABORT:
            if self.axial_distance(state[:3]) < self.guidance.port_alignment_range:
                self.pointing_mode = "port"

        if self.phase in _HOLD_PHASES:
            return self._hold(t, state)
        if self.phase == Phase.TRANSFER:
            return self._transfer(t, state)
        if self.phase == Phase.APPROACH:
            return self._approach(t, state)
        if self.phase == Phase.FINAL_APPROACH:
            return self._final_approach(t, state)
        if self.phase == Phase.ABORT:
            return self._abort(t)

        return np.zeros(3)

    def _control_due(self, t):
        if t + 1e-9 >= self._next_control:
            self._next_control = t + self.control.translation_period
            return True
        return False

    def _limit(self, acceleration):
        """Scale a command so that no axis exceeds the available acceleration."""

        largest = np.max(np.abs(acceleration))
        if largest > self.max_acceleration:
            return acceleration * (self.max_acceleration / largest)
        return acceleration

    def _burn_command(self):
        """Acceleration that delivers the remaining velocity change as fast as possible."""

        remaining = np.sqrt(self._burn @ self._burn)
        if remaining < 5e-4:
            self._burn = None
            return np.zeros(3)

        return self._burn / remaining * min(self.max_acceleration, remaining / self.dt)

    # ------------------------------------------------------------------
    # phases
    # ------------------------------------------------------------------

    def _hold(self, t, state):
        index = _HOLD_PHASES.index(self.phase)
        reference = self.hold_positions[index]
        g = self.guidance

        if self._control_due(t):
            if index < 2:
                self._command = self._limit(self.hold_controllers[index].acceleration(state, reference))
            else:
                self._command = self._predictive_command(
                    self.final_mpc, state, np.tile(np.concatenate([reference, np.zeros(3)]),
                                                   (self.control.mpc_horizon + 1, 1)),
                    None, final=False,
                )

        position_error = np.linalg.norm(state[:3] - reference)
        speed = np.linalg.norm(state[3:])
        settled = (
            t - self.phase_start >= g.hold_durations[index]
            and position_error <= g.hold_position_tolerance[index]
            and speed <= g.hold_velocity_tolerance[index]
        )

        if settled:
            if index == 0:
                self._start_transfer(t, state)
            elif index == 1:
                self._carrot = GlideslopeReference(
                    reference, self.hold_positions[2], g.approach_v_max, g.approach_v_min,
                    g.approach_time_constant, g.approach_acceleration, brake=True,
                )
                self._enter(Phase.APPROACH, t)
            elif self._go_for_final(t):
                end = self.contact_position - g.docking_overtravel * self.axis
                self._carrot = GlideslopeReference(
                    reference, end, g.docking_speed, g.docking_speed, 1.0,
                    g.approach_acceleration, brake=False,
                )
                self._enter(Phase.FINAL_APPROACH, t)

        return self._command

    def _go_for_final(self, t):
        g = self.guidance
        if not g.lighting_gate or self.lighting_check is None:
            return True

        duration = g.hold_distances[2] / g.docking_speed + g.lighting_margin
        return self.lighting_check(t, duration)

    def _start_transfer(self, t, state):
        g = self.guidance
        time_of_flight = g.transfer_orbit_fraction * 2.0 * np.pi / self.n
        dv, _ = plan_transfer(state, self.hold_positions[1], time_of_flight, self.n, self.max_acceleration)

        self._enter(Phase.TRANSFER, t, f"departure burn {np.linalg.norm(dv):.3f} m/s")
        self._burn = dv
        self._arrival_time = t + time_of_flight
        self._corrections = [t + f * time_of_flight for f in g.correction_fractions]

    def _transfer(self, t, state):
        time_to_go = self._arrival_time - t
        speed = np.linalg.norm(state[3:])

        if self._braking:
            if speed <= self.guidance.hold_velocity_tolerance[1] or t - self._brake_start > 600.0:
                self._enter(Phase.HOLD_MID, t)
                return np.zeros(3)
            return -state[3:] / speed * min(self.max_acceleration, speed / self.dt)

        if time_to_go <= 0.5 * speed / self.max_acceleration + self.dt:
            self._braking = True
            self._brake_start = t
            self._burn = None
            self.events.append((t, Phase.TRANSFER, f"arrival burn {speed:.3f} m/s"))
            return -state[3:] / speed * min(self.max_acceleration, speed / self.dt)

        if self._burn is None and self._corrections and t >= self._corrections[0]:
            self._corrections.pop(0)
            dv, _ = cw_targeting(state[:3], state[3:], self.hold_positions[1], np.zeros(3), time_to_go, self.n)
            if np.linalg.norm(dv) > self.guidance.correction_threshold:
                self._burn = dv
                self.events.append((t, Phase.TRANSFER, f"correction burn {np.linalg.norm(dv):.3f} m/s"))

        if self._burn is not None:
            return self._burn_command()

        return np.zeros(3)

    def _predictive_command(self, controller, state, reference, feedforward, final):
        bound = self._constraint_offset.copy()
        if final:
            # during the final approach the deputy is meant to reach the port
            bound[5] += 10.0

        command = controller.solve(state, reference, feedforward, bound)
        self.mpc_solves += 1
        if command is None:
            # the solver did not answer: fall back on the regulator
            self.mpc_fallbacks += 1
            first = np.zeros(3) if feedforward is None else feedforward[0]
            command = self._limit(first - controller.lqr_gain @ (state - reference[0]))
        else:
            self.mpc_max_slack = max(self.mpc_max_slack, controller.last_slack)

        return command

    def _approach(self, t, state):
        g = self.guidance
        self._carrot.advance(self.dt)

        if self._control_due(t):
            reference, feedforward = self._carrot.horizon(
                self.control.translation_period, self.control.mpc_horizon, self.n,
            )
            self._command = self._predictive_command(self.approach_mpc, state, reference, feedforward, final=False)

        if self._carrot.arrived:
            position_error = np.linalg.norm(state[:3] - self.hold_positions[2])
            if position_error <= 2.0 * g.hold_position_tolerance[2] + 0.5:
                self._enter(Phase.HOLD_NEAR, t)

        return self._command

    def _final_approach(self, t, state):
        self._carrot.advance(self.dt)

        if self._carrot.arrived:
            # the reference went past the port and no contact was reported
            self._start_abort(t, "no contact at the expected position")
            return self._abort(t)

        if self._control_due(t):
            reference, feedforward = self._carrot.horizon(
                self.control.translation_period, self.control.mpc_horizon, self.n,
            )
            self._command = self._predictive_command(self.final_mpc, state, reference, feedforward, final=True)

        return self._command

    # ------------------------------------------------------------------
    # abort
    # ------------------------------------------------------------------

    def _abort_condition(self, state, position_std, measurement_age, pointing_error):
        g = self.guidance
        distance = max(self.axial_distance(state[:3]), 0.0)

        allowed = g.abort_corridor_radius + np.tan(np.deg2rad(g.abort_corridor_half_angle_deg)) * distance
        if self.lateral_distance(state[:3]) > allowed:
            return "outside the approach corridor"

        closing_speed = -(state[3:] @ self.axis)
        if closing_speed > g.abort_closing_speed_factor * (g.closing_speed_floor + g.closing_speed_slope * distance):
            return "closing speed too high"

        if measurement_age > g.abort_measurement_outage:
            return "relative measurements lost"

        if np.max(position_std) > max(g.abort_position_std_min, g.abort_position_std_fraction * distance):
            return "navigation uncertainty too large"

        if pointing_error > np.deg2rad(g.abort_pointing_error_deg):
            return "attitude error too large"

        return None

    def _start_abort(self, t, reason):
        self.abort_reason = reason
        self._enter(Phase.ABORT, t, reason)
        self._burn = self.guidance.abort_delta_v * self.axis
        self._abort_end = t + self.guidance.abort_coast_duration
        self.pointing_mode = "target"

    def _abort(self, t):
        if t >= self._abort_end:
            self.finished = True

        if self._burn is not None:
            return self._burn_command()

        return np.zeros(3)
