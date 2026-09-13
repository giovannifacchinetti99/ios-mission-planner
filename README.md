# ios-mission-planner

Tools for in-orbit servicing (IOS) mission analysis and planning: relative orbital motion dynamics and propagation between a chief and a deputy satellite.

## Contents

- `src/ios_mission_planner/dynamics/hill.py` — Hill-Clohessy-Wiltshire (HCW) equations of motion for a deputy relative to a chief on a circular reference orbit, for use with numerical integration.
- `src/ios_mission_planner/dynamics/cw.py` — Closed-form Clohessy-Wiltshire state transition matrix for analytical propagation of relative motion.
- `src/ios_mission_planner/propagation/propagator.py` — Generic numerical propagator built on `scipy.integrate.solve_ivp`.
- `examples/cw_comparison.ipynb` — Notebook comparing numerical (HCW) and analytical (CW state transition matrix) propagation.

## Installation

```bash
pip install -e .
```

Requires Python >= 3.12.

## Usage

```python
import numpy as np
from ios_mission_planner.dynamics.hill import hill_equations
from ios_mission_planner.propagation.propagator import propagate

n = 0.0011  # mean motion [rad/s]
state0 = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])
t_span = (0, 5400)
t_eval = np.linspace(*t_span, 200)

solution = propagate(hill_equations, state0, t_span, t_eval, n=n)
```
