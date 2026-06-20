"""Exploration-policy interface (abstract base).

The turn decision is ``argmax`` over candidate headings of

    heading_score = policy.heading_preference  (positive / preferred)
                    - physical_constraints     (negative penalties)

``heading_score`` (in ``map_room``) owns the physical-constraint penalties.
All *positive* preference scoring lives in an ``ExplorationPolicy``. Concrete
policies override ``heading_preference``; every other hook has a safe,
unconstrained default so an unconstrained policy only implements the reward.

Each concrete policy lives in its own module (1 file, 1 class) -- e.g. the
default ``NoveltyExplorationPolicy`` in ``novelty_exploration.py``; this module
holds only the abstract base they share.
"""

from abc import ABC, abstractmethod


class ExplorationPolicy(ABC):
    """Pluggable source of positive heading preference for the explorer.

    Defaults describe an *unconstrained* policy: the whole plane is allowed,
    forward legs are never clamped, nothing unlocks or expands, and no progress
    is reported. Subclasses that add a constraint (e.g. bounded territories)
    override the relevant hooks.
    """

    #: key under which this policy's per-run metadata is stored in the map JSON.
    metadata_key = 'exploration_policy'

    @abstractmethod
    def heading_preference(self, x, y, heading):
        """Return the positive score for facing ``heading`` from ``(x, y)``."""

    def allows_point(self, x, y):
        """Whether the robot is permitted at ``(x, y)`` (no limit by default)."""
        return True

    def forward_distance(self, x, y, heading, desired_distance):
        """Clamp a forward leg to what the policy allows (no clamp by default)."""
        return desired_distance

    def unlock_if_complete(self):
        """Advance policy state once per turn decision (no-op by default)."""

    def expand_past_boundary(self, x, y, heading):
        """Try to grow the allowed region ahead (never, by default)."""
        return False

    def add_blocked_territory_expansion(self, x, y, heading):
        """Record a physical stop during expansion (no-op by default)."""

    def is_complete(self):
        """Whether exploration has exhausted the policy objective."""
        return False

    def report_progress(self):
        """Emit progress after an accepted leg or observation (no-op)."""

    def describe(self):
        """One-line summary printed when exploration starts."""
        return "  Novelty exploration: undirected forward legs, no territory limit."

    def metadata(self):
        """Per-run metadata recorded in the map JSON."""
        return {}


# Command policies emit an explicit move/turn course (see preset_exploration);
# they are a separate family from the heading policies above, registered here so
# load_exploration_policy can resolve a config by name.
try:
    from apps.map.policies.preset_exploration import PresetExplorationPolicy
except ModuleNotFoundError:
    from policies.preset_exploration import PresetExplorationPolicy


COMMAND_POLICIES = {
    PresetExplorationPolicy.name: PresetExplorationPolicy,
}


def load_exploration_policy(configs):
    """Load the first configured command policy, which has command priority."""
    if not configs:
        return None
    config = configs[0]
    policy_type = COMMAND_POLICIES.get(config['name'])
    if policy_type is None:
        raise ValueError(f"unknown exploration policy: {config['name']}")
    return policy_type.from_config(config)
