from dataclasses import dataclass, field

import yaml


@dataclass
class ChiefScenario:
    altitude_km: float
    inclination_deg: float


@dataclass
class PropulsionScenario:
    isp_s: float


@dataclass
class DeputyScenario:
    parking_altitude_km: float
    dry_mass_kg: float
    propulsion: PropulsionScenario


@dataclass
class PhasingScenario:
    hold_range_km: float


@dataclass
class SafetyEllipseScenario:
    radius_m: float
    hop_duration_orbits: float


@dataclass
class StationKeepingScenario:
    rate_m_s_per_orbit: float
    dwell_days: float


@dataclass
class DockingScenario:
    duration_orbits: float
    closing_speed_m_s: float


@dataclass
class DisposalScenario:
    target_perigee_altitude_km: float


@dataclass
class MonteCarloScenario:
    nav_sigma_m: float
    burn_sigma_m_s: float
    n_samples: int = 20000


@dataclass
class SafetyChecksScenario:
    keep_out_zone_m: tuple
    approach_corridor_half_angle_deg: float
    approach_corridor_direction: tuple
    passive_safety_duration_s: float
    monte_carlo: MonteCarloScenario


@dataclass
class MarginsScenario:
    system_reserve: float = 0.10


@dataclass
class VerdictThresholds:
    max_collision_probability: float
    max_wet_mass_kg: float


@dataclass
class MissionScenario:
    """
    A declarative description of an in-orbit servicing mission, loadable
    from a YAML file, for the Layer 4 feasibility pipeline.

    This is deliberately a simplified mission template, not a full mission
    designer: it covers a single-plane (or single-plane-change) rendezvous
    from a parking orbit to a 2:1 safety ellipse, a station-keeping dwell,
    docking, and a deorbit disposal burn, the same mission shape built up
    across examples/03_rendezvous_to_safety_ellipse through
    examples/08_mission_budget. A mission that does not fit that shape
    needs a different pipeline, not just a different scenario file.

    Author: Giovanni Facchinetti, 2026
    """

    name: str
    chief: ChiefScenario
    deputy: DeputyScenario
    phasing: PhasingScenario
    safety_ellipse: SafetyEllipseScenario
    station_keeping: StationKeepingScenario
    docking: DockingScenario
    disposal: DisposalScenario
    safety_checks: SafetyChecksScenario
    verdict_thresholds: VerdictThresholds
    margins: MarginsScenario = field(default_factory=MarginsScenario)
    plane_change_deg: float = 0.0


def load_scenario(path):
    """
    Load a MissionScenario from a YAML file.

    Parameters
    ----------
    path : str or pathlib.Path

    Returns
    -------
    MissionScenario

    Author: Giovanni Facchinetti, 2026
    """

    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    return MissionScenario(
        name=raw["name"],
        chief=ChiefScenario(**raw["chief"]),
        deputy=DeputyScenario(
            parking_altitude_km=raw["deputy"]["parking_altitude_km"],
            dry_mass_kg=raw["deputy"]["dry_mass_kg"],
            propulsion=PropulsionScenario(**raw["deputy"]["propulsion"]),
        ),
        phasing=PhasingScenario(**raw["phasing"]),
        safety_ellipse=SafetyEllipseScenario(**raw["safety_ellipse"]),
        station_keeping=StationKeepingScenario(**raw["station_keeping"]),
        docking=DockingScenario(**raw["docking"]),
        disposal=DisposalScenario(**raw["disposal"]),
        safety_checks=SafetyChecksScenario(
            keep_out_zone_m=tuple(raw["safety_checks"]["keep_out_zone_m"]),
            approach_corridor_half_angle_deg=raw["safety_checks"]["approach_corridor_half_angle_deg"],
            approach_corridor_direction=tuple(raw["safety_checks"]["approach_corridor_direction"]),
            passive_safety_duration_s=raw["safety_checks"]["passive_safety_duration_s"],
            monte_carlo=MonteCarloScenario(**raw["safety_checks"]["monte_carlo"]),
        ),
        verdict_thresholds=VerdictThresholds(**raw["verdict_thresholds"]),
        margins=MarginsScenario(**raw.get("margins", {})),
        plane_change_deg=raw.get("plane_change_deg", 0.0),
    )
