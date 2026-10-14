from dataclasses import dataclass

# Typical maturity-based margins on a delta-v line item, in the spirit of
# ECSS/ESA concurrent-design-facility budgeting practice: less margin on a
# maneuver backed by an established, validated model, more on one that is
# still preliminary. These are reasonable defaults, not fixed requirements;
# BudgetLine takes a plain fractional margin either way.
MATURITY_MARGIN = {
    "flight_proven": 0.02,
    "analyzed": 0.10,
    "preliminary": 0.20,
}


@dataclass
class BudgetLine:
    """
    One delta-v line item in a mission budget, with its own margin.

    Attributes
    ----------
    label : str
    delta_v : float
        Nominal (unmargined) delta-v [m/s].
    margin : float
        Fractional margin on top of delta_v, e.g. 0.10 for 10%. Pick a
        value directly or look one up in MATURITY_MARGIN by how mature the
        maneuver's design is.

    Author: Giovanni Facchinetti, 2026
    """

    label: str
    delta_v: float
    margin: float

    @property
    def margined_delta_v(self):
        """Delta-v for this line including its own margin [m/s]."""
        return self.delta_v * (1.0 + self.margin)


def total_delta_v(lines, system_margin=0.0):
    """
    Total delta-v budget: the margined line items, summed, plus an
    overall system-level reserve on top of that subtotal.

    Parameters
    ----------
    lines : list[BudgetLine]
    system_margin : float
        Fractional reserve applied to the summed, already-margined
        subtotal, e.g. 0.10 for a 10% mission reserve.

    Returns
    -------
    float
        Total delta-v [m/s], margined line items plus system reserve.

    Author: Giovanni Facchinetti, 2026
    """

    subtotal = sum(line.margined_delta_v for line in lines)
    return subtotal * (1.0 + system_margin)
