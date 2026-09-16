# ios-mission-planner

Tools for in-orbit servicing (IOS) mission analysis and planning: relative orbital motion dynamics and propagation between a chief and a deputy satellite.

## Contents

- `src/ios_mission_planner/dynamics/orbital/two_body.py` — Two-body equations of motion in an inertial frame, for absolute orbit propagation.
- `src/ios_mission_planner/dynamics/orbital/perturbations.py` — Two-body dynamics with J2 (oblateness) perturbation.
- `src/ios_mission_planner/dynamics/orbital/elements.py` — Conversions between inertial state vectors and classical orbital elements, and Kepler's equation solver.
- `src/ios_mission_planner/dynamics/orbital/kepler_propagator.py` — Closed-form (Keplerian) analytical propagation of the unperturbed two-body state.
- `src/ios_mission_planner/dynamics/relative/hill.py` — Hill-Clohessy-Wiltshire (HCW) equations of motion for a deputy relative to a chief on a circular reference orbit, for use with numerical integration.
- `src/ios_mission_planner/dynamics/relative/cw.py` — Closed-form Clohessy-Wiltshire state transition matrix for analytical propagation of relative motion.
- `src/ios_mission_planner/propagation/propagator.py` — Generic numerical propagator built on `scipy.integrate.solve_ivp`.
- `src/ios_mission_planner/constants.py` — Physical constants (gravitational parameter, radius, J2) for common central bodies.
- `examples/cw_comparison.ipynb` — Notebook comparing numerical (HCW) and analytical (CW state transition matrix) propagation.

## Installation

```bash
pip install -e .
```

Requires Python >= 3.12.

## Usage

Relative motion (HCW), numerically integrated:

```python
import numpy as np
from ios_mission_planner.dynamics.relative.hill import hill_equations
from ios_mission_planner.propagation.propagator import propagate

n = 0.0011  # mean motion [rad/s]
state0 = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])
t_span = (0, 5400)
t_eval = np.linspace(*t_span, 200)

solution = propagate(hill_equations, state0, t_span, t_eval, n=n)
```

Absolute (inertial) two-body motion, numerically integrated:

```python
import numpy as np
from ios_mission_planner.constants import MU_EARTH
from ios_mission_planner.dynamics.orbital.two_body import two_body_dynamics
from ios_mission_planner.propagation.propagator import propagate

state0 = np.array([6878.137, 0.0, 0.0, 0.0, 7.6126, 0.0])  # km, km/s
t_span = (0, 5677)
t_eval = np.linspace(*t_span, 200)

solution = propagate(two_body_dynamics, state0, t_span, t_eval, mu=MU_EARTH)
```

Absolute two-body motion, propagated analytically (Kepler):

```python
from ios_mission_planner.constants import MU_EARTH
from ios_mission_planner.dynamics.orbital.kepler_propagator import propagate_kepler

state_t = propagate_kepler(state0, t=5677.0, mu=MU_EARTH)
```
