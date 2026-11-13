# ios-mission-planner

A Python toolkit for **in-orbit servicing (IOS) mission analysis**: it designs the maneuvers that take a servicer spacecraft from a parking orbit to a safe rendezvous with its target, flies them against nonlinear orbital dynamics instead of the linear model used to design them, checks the result for collision and safety risk, simulates the approach and the docking in a closed loop with sensors, navigation, guidance, control and actuators, estimates the probability of success with Monte Carlo campaigns, finds when the docking port is in the light, and turns the whole mission into a propellant budget, a report and a 3D replay.

**[>> Read the documentation](https://giovannifacchinetti99.github.io/ios-mission-planner/)**

The documentation is a book that follows one reference mission from the first principles to the complete analysis. Each chapter gives the theory, describes the functions that implement it, and ends with a notebook that validates the model against an independent reference.

![The 3D viewer during the final approach](docs/images/docking_viewer.png)

*The final approach of the closed-loop simulation in the 3D viewer, with the approach corridor and the field of view of the relative sensor, from [`docking_3d_viewer.ipynb`](examples/21_docking_3d_viewer/docking_3d_viewer.ipynb). The viewer itself is [in the documentation](https://giovannifacchinetti99.github.io/ios-mission-planner/visualization/viewer.html).*

![The closed-loop approach and its navigation error](docs/images/closed_loop_docking.png)

*The approach from 5 km to contact flown by the on-board chain against a nonlinear truth model, and the error of the relative navigation against the three-sigma bound of its filter, from [`closed_loop_docking.ipynb`](examples/15_closed_loop_docking/closed_loop_docking.ipynb).*

![The far-range approach and the resulting safety ellipse](docs/images/rendezvous_safety_ellipse.png)

*The approach from a hold point 5 km behind the target onto a closed 2:1 ellipse, designed with the linear Clohessy-Wiltshire model and flown with nonlinear two-body dynamics, from [`rendezvous_to_safety_ellipse.ipynb`](examples/06_rendezvous_to_safety_ellipse/rendezvous_to_safety_ellipse.ipynb).*

![Safety-checking the whole mission against a keep-out zone](docs/images/mission_safety_check.png)

*Range to the target over the whole rendezvous, checked against a keep-out zone, from [`rendezvous_safety_assessment.ipynb`](examples/09_rendezvous_safety_assessment/rendezvous_safety_assessment.ipynb). The same notebook checks passive safety, the approach corridor, the effect of J2 and the collision probability.*

![Delta-v budget by mission phase](docs/images/mission_budget.png)

*The mission from phasing to disposal as a margined delta-v budget, from [`mission_budget.ipynb`](examples/16_mission_budget/mission_budget.ipynb).*

## Contents

The package and the examples follow the order of the book. The notebooks under `examples/` are numbered in reading order.

| Part | Topic | Package | Notebooks |
| --- | --- | --- | --- |
| I | Two-body motion, orbital elements, J2 and drag | `dynamics.orbital` | 01 |
| II | Relative motion: Clohessy-Wiltshire, eccentric chief, relative orbital elements under J2, differential drag | `dynamics.relative` | 02 to 05 |
| III | Impulsive maneuvers, two-impulse targeting, rocket equation, low thrust with pygmo optimization | `dynamics.orbital`, `dynamics.relative`, `budget` | 06, 07 |
| IV | Keep-out zones, passive safety, approach corridors, collision probability | `safety` | 08, 09 |
| V | Attitude dynamics and control, TRIAD and Wahba determination, MEKF, target pointing | `dynamics.attitude` | 10 to 13 |
| VI | Closed-loop simulation of approach and docking: six-degree-of-freedom truth model, sensors, relative navigation filter, hold points, two-impulse transfer, glideslope, LQR and model predictive control, pulsed thrusters, reaction wheels and momentum management, collision avoidance maneuver | `sim`, `gnc` | 14, 15 |
| VII | Mission budget with margins, Monte Carlo campaigns of the closed loop, illumination and lighting windows of the docking port, automated reports, a real proximity event reconstructed from public data | `budget`, `sim`, `illumination`, `report` | 16 to 20 |
| VIII | 3D viewer that replays a simulation in the browser | `viz` | 21 |

## Installation

Requires [conda](https://conda.io) (for example [Miniforge](https://github.com/conda-forge/miniforge)). The dynamics modules depend on [heyoka.py](https://github.com/bluescarni/heyoka.py), which is distributed through conda-forge and has no pip wheels for Windows, so the project is installed with conda.

```bash
git clone https://github.com/giovannifacchinetti99/ios-mission-planner.git
cd ios-mission-planner

conda env create -f environment.yml
conda activate ios-mission-planner
```

`environment.yml` installs Python, heyoka.py, numpy, scipy, matplotlib, Jupyter, [pygmo](https://esa.github.io/pygmo2/) and [OSQP](https://osqp.org/) for optimization, PyYAML, Jinja2 and [WeasyPrint](https://weasyprint.org/) for the reports, and the package itself in editable mode, so that changes under `src/` are picked up without reinstalling.

To run the notebooks with the kernel of this environment:

```bash
python -m ipykernel install --user --name ios-mission-planner --display-name "Python (ios-mission-planner)"
```

## Testing

```bash
pytest
```

The suite under `tests/` covers every module in `src/`: conservation laws and round trips for the dynamics, comparisons of closed-form results with numerical propagation, sign conventions for the safety tools, the statistics of the sensor and actuator models, the consistency of the navigation filter, the constraints of the predictive controller, and the closed loop itself, from a nominal docking to injected failures.

## Usage

Propagating the Hill-Clohessy-Wiltshire equations:

```python
import numpy as np
from ios_mission_planner.dynamics.relative.hill import hill_equations_symbolic
from ios_mission_planner.propagation.heyoka_propagator import propagate

n = 0.0011  # mean motion of the chief [rad/s]
state0 = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # relative state [m, m/s]
t_eval = np.linspace(0, 5400, 200)

solution = propagate(hill_equations_symbolic(), state0, t_eval, pars=[n])
```

Two-impulse targeting between two hold points on the V-bar, in three quarters of an orbit:

```python
import numpy as np
from ios_mission_planner.dynamics.relative.cw import cw_targeting

n = 0.0011
T = 2 * np.pi / n

dv1, dv2 = cw_targeting(
    r0=[0.0, -1000.0, 0.0], v0=[0.0, 0.0, 0.0],
    rf=[0.0, -200.0, 0.0], vf=[0.0, 0.0, 0.0],
    t=0.75 * T, n=n,
)
```

Checking a relative trajectory against a keep-out zone:

```python
import numpy as np
from ios_mission_planner.safety.keepout import KeepOutEllipsoid, check_trajectory

zone = KeepOutEllipsoid(a=20.0, b=20.0, c=20.0)  # metres, centered on the chief
positions = np.array([[100.0, 0.0, 0.0], [15.0, 0.0, 0.0]])

violated, margin, mask = check_trajectory(positions, zone)
```

A margined delta-v budget converted to propellant:

```python
from ios_mission_planner.budget.margins import BudgetLine, MATURITY_MARGIN, total_delta_v
from ios_mission_planner.budget.propellant import initial_mass

lines = [
    BudgetLine("phasing and transfer", 55.9, MATURITY_MARGIN["analyzed"]),
    BudgetLine("station keeping", 13.3, MATURITY_MARGIN["preliminary"]),
]
dv_total = total_delta_v(lines, system_margin=0.10)

propellant_kg = initial_mass(dv_total, isp=220.0, m_dry=150.0) - 150.0
```

A feasibility screening and report from a YAML scenario:

```python
from ios_mission_planner.report.scenario import load_scenario
from ios_mission_planner.report.pipeline import run_feasibility_analysis
from ios_mission_planner.report.render import render_html, render_pdf

scenario = load_scenario("my_mission.yaml")
result = run_feasibility_analysis(scenario)

print(result.verdict)  # "FEASIBLE" / "FEASIBLE WITH CAVEATS" / "NOT FEASIBLE"

html = render_html(result)
render_pdf(html, "report.pdf")
```

A closed-loop simulation of the approach and docking, a Monte Carlo campaign and the 3D viewer:

```python
from ios_mission_planner.sim.config import SimulationConfig
from ios_mission_planner.sim.monte_carlo import Dispersions, run_monte_carlo
from ios_mission_planner.sim.simulator import simulate
from ios_mission_planner.viz.viewer import export_viewer

config = SimulationConfig()          # the reference mission; every parameter can be changed
result = simulate(config)

print(result.outcome)                # "docked", "aborted", ...
print(result.contact.values)         # conditions at contact, against the limits of the mechanism
print(result.delta_v_by_phase())

export_viewer(result, "docking.html")   # a self-contained page, opens in any browser

campaign = run_monte_carlo(config, Dispersions(), n_runs=200, workers=6)
print(campaign.success_rate, campaign.success_interval(0.95))
```

On Windows a script that runs a campaign on several processes must do so under `if __name__ == "__main__":`.

The lighting windows of a docking port:

```python
import numpy as np
from ios_mission_planner.constants import J2_EARTH, MU_EARTH, R_EARTH
from ios_mission_planner.illumination.lighting import port_lighting_timeline
from ios_mission_planner.illumination.sun import julian_date

timeline = port_lighting_timeline(
    radius=R_EARTH + 500.0, inclination=np.deg2rad(51.6), raan=np.deg2rad(40.0), argument_of_latitude=0.0,
    jd_epoch=julian_date(2026, 3, 20, 12), duration=3 * 5677.0, step=10.0,
    mu=MU_EARTH, j2=J2_EARTH, r_eq=R_EARTH, port_normal_lvlh=(0.0, -1.0, 0.0),
)
print(timeline.windows())            # (start, end) of each favourable interval [s]
```

## Building the documentation

The book under `docs/book/` is built with Jupyter Book and published automatically on every push; see [`docs/README.md`](docs/README.md).
