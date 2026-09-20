# ios-mission-planner

A Python toolkit for **in-orbit servicing (IOS) mission analysis and planning**.

The long-term goal is a mission planner that, given a deputy satellite on a parking orbit and a target (chief) satellite, computes an optimal sequence of maneuvers to bring the deputy from far-range phasing through close-range proximity operations to a final docking — accounting for orbital and attitude dynamics of both satellites, mission safety constraints (e.g. approach/keep-out ellipsoids), and eventually closed-loop relative navigation from noisy sensor data (e.g. LiDAR).

This is being built up incrementally, starting from the dynamics layer. What exists today:

- **Relative motion** between a chief and a deputy: the linearized Hill-Clohessy-Wiltshire (HCW) equations, both integrated numerically and solved in closed form via the Clohessy-Wiltshire state transition matrix (STM).
- **Absolute (inertial) motion**: the two-body problem plus the J2 (Earth oblateness) perturbation.
- **Frame transformations** between the inertial frame and the chief's LVLH frame, so that maneuvers designed with linear relative-motion tools can be flown and checked against nonlinear orbits.
- **Relative-motion planning tools**: two-impulse CW targeting (the delta-v to go from one relative state to another in a fixed time), and a geometric decomposition of any relative orbit into a drift-free 2:1 ellipse plus drift, the basis for passively safe parking orbits.
- **Numerical propagation** via [heyoka.py](https://github.com/bluescarni/heyoka.py): equations of motion are defined as symbolic expressions and integrated with an adaptive-order Taylor series, JIT-compiled via LLVM, instead of a fixed-order Runge-Kutta scheme.

![Natural motion circumnavigation — CW analytical vs. numerical](docs/images/cw_natural_motion_circumnavigation.png)

*A drift-free, out-of-plane relative orbit (natural motion circumnavigation) around the chief, from [`examples/cw_comparison.ipynb`](examples/cw_comparison.ipynb): the CW closed-form solution and the numerical integration of the Hill equations agree down to numerical noise (bottom-right panel).*

## Installation

Requires [conda](https://conda.io) (e.g. [Miniforge](https://github.com/conda-forge/miniforge)). The dynamics modules depend on [heyoka.py](https://github.com/bluescarni/heyoka.py), which ships prebuilt binaries only through conda-forge (no Windows pip wheels), so this project is conda-first rather than pip/venv-based.

```bash
git clone https://github.com/giovannifacchinetti99/ios-mission-planner.git
cd ios-mission-planner

conda env create -f environment.yml
conda activate ios-mission-planner
```

`environment.yml` installs Python, heyoka.py, numpy, matplotlib, Jupyter, and the package itself in editable mode (via `pip install -e .` under the hood), so changes to `src/` are picked up immediately without reinstalling.

To run the example notebooks from this environment's Jupyter kernel:

```bash
python -m ipykernel install --user --name ios-mission-planner --display-name "Python (ios-mission-planner)"
```

## Contents

- `src/ios_mission_planner/dynamics/orbital/two_body.py` — Two-body equations of motion in an inertial frame, as a symbolic heyoka ODE system, for absolute orbit propagation.
- `src/ios_mission_planner/dynamics/orbital/perturbations.py` — Two-body dynamics with J2 (oblateness) perturbation, as a symbolic heyoka ODE system.
- `src/ios_mission_planner/dynamics/relative/hill.py` — Hill-Clohessy-Wiltshire (HCW) equations of motion for a deputy relative to a chief on a circular reference orbit, as a symbolic heyoka ODE system.
- `src/ios_mission_planner/dynamics/relative/cw.py` — Closed-form Clohessy-Wiltshire state transition matrix for analytical propagation of relative motion, and two-impulse targeting built on it.
- `src/ios_mission_planner/dynamics/relative/relative_orbit.py` — Conversion between a relative state and its geometric description (2:1 ellipse size, phase, center, drift), and the drift-free ellipse through a given point.
- `src/ios_mission_planner/dynamics/relative/lvlh.py` — Inertial <-> chief LVLH frame transformations of position and velocity.
- `src/ios_mission_planner/propagation/heyoka_propagator.py` — Generic numerical propagator built on heyoka.py's Taylor-adaptive integrator.
- `src/ios_mission_planner/constants.py` — Physical constants (gravitational parameter, radius, J2) for common central bodies.
- `examples/cw_comparison.ipynb` — Numerical vs. analytical comparison of relative motion (HCW/CW), across several representative cases (drift-free ellipse, V-bar hold point, natural motion circumnavigation, ...), including a walkthrough of how heyoka is used.
- `examples/cw_targeting_and_relative_orbits.ipynb` — Relative orbit elements and two-impulse targeting, ending with the insertion of a deputy onto a passively safe 2:1 ellipse, verified by integrating the Hill equations with heyoka.
- `examples/rendezvous_to_safety_ellipse.ipynb` — A complete approach: parking orbit, phasing, Hohmann to a 5 km hold point, far-range CW hop and insertion onto a 2:1 safety ellipse, flown with nonlinear two-body dynamics, with a delta-v budget and a J2 experiment.

## Usage

Relative motion (HCW), numerically integrated:

```python
import numpy as np
from ios_mission_planner.dynamics.relative.hill import hill_equations_symbolic
from ios_mission_planner.propagation.heyoka_propagator import propagate

n = 0.0011  # mean motion [rad/s]
state0 = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])
t_eval = np.linspace(0, 5400, 200)

solution = propagate(hill_equations_symbolic(), state0, t_eval, pars=[n])
```

Absolute (inertial) two-body motion, numerically integrated:

```python
import numpy as np
from ios_mission_planner.constants import MU_EARTH
from ios_mission_planner.dynamics.orbital.two_body import two_body_dynamics_symbolic
from ios_mission_planner.propagation.heyoka_propagator import propagate

state0 = np.array([6878.137, 0.0, 0.0, 0.0, 7.6126, 0.0])  # km, km/s
t_eval = np.linspace(0, 5677, 200)

solution = propagate(two_body_dynamics_symbolic(), state0, t_eval, pars=[MU_EARTH])
```

Two-impulse targeting from a hold point at y = -1000 m to one at y = -200 m, in 0.75 orbits:

```python
import numpy as np
from ios_mission_planner.dynamics.relative.cw import cw_targeting

n = 0.0011  # mean motion [rad/s]
T = 2 * np.pi / n

dv1, dv2 = cw_targeting(
    r0=[0.0, -1000.0, 0.0], v0=[0.0, 0.0, 0.0],
    rf=[0.0, -200.0, 0.0], vf=[0.0, 0.0, 0.0],
    t=0.75 * T, n=n,
)
```
