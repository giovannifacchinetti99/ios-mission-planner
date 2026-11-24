"""
Interface between the closed-loop simulation and an external supervisor.

A supervisor is a second layer of on-board logic that watches the
spacecraft and may overrule the main guidance, navigation and control
chain, such as the independent collision avoidance units flown on cargo
vehicles. It is plugged into ios_mission_planner.sim.simulator.simulate
as a callable. Once per step, after the main chain has computed its force
command and before the thrusters execute it, the simulation calls

    decision = supervisor(sample)

with a SupervisorSample, and the supervisor returns None, to let the main
chain act, or a SupervisorDecision.

The sample holds only the truth and the force command, never the
estimates, covariances or internal state of the main chain. This is
deliberate: a supervisor meant to be independent of the main navigation
has to build its own knowledge, typically by synthesising the
measurements of its own sensor from the true state with its own errors.
"""

from dataclasses import dataclass

import numpy as np


def _read_only(values):
    array = np.array(values, dtype=float)
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class SupervisorSample:
    """
    What the supervisor receives at each step.

    Attributes
    ----------
    t : float
        Time since the start of the simulation [s].
    relative_state : numpy.ndarray, shape (6,)
        True position and velocity of the deputy in the LVLH frame of the
        chief [m, m/s].
    body_to_lvlh : numpy.ndarray, shape (3, 3)
        True attitude of the deputy: the matrix that turns components in
        its body frame into components in the LVLH frame.
    force_command : numpy.ndarray, shape (3,)
        Force that the main chain is about to command, body frame [N].

    The arrays are read-only copies.

    Author: Giovanni Facchinetti, 2026
    """

    t: float
    relative_state: np.ndarray
    body_to_lvlh: np.ndarray
    force_command: np.ndarray

    @classmethod
    def build(cls, t, relative_state, body_to_lvlh, force_command):
        return cls(float(t), _read_only(relative_state), _read_only(body_to_lvlh), _read_only(force_command))


@dataclass(frozen=True)
class SupervisorDecision:
    """
    What a supervisor may ask for at one step.

    Attributes
    ----------
    force_command : numpy.ndarray or None
        Force to command instead of the main chain's, body frame [N]. It
        goes to the same thrusters, with the same limits and errors. None
        keeps the main chain's command.
    request_abort : bool
        Ask the main chain to start its collision avoidance maneuver. The
        mission manager enters the abort phase with the reason
        "supervisor request" and flies its own maneuver from the next
        step on, unless the supervisor keeps replacing the force command.
    note : str
        Short description, recorded with the events of the simulation when
        the decision changes.

    Author: Giovanni Facchinetti, 2026
    """

    force_command: np.ndarray = None
    request_abort: bool = False
    note: str = ""

    @property
    def overrides_force(self):
        return self.force_command is not None
