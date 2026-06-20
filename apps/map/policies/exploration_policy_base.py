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

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass
class PolicyContext:
    """A run's live state, packaged so a policy can build itself from it.

    The run fills one of these in and hands it to ``Policy.from_context``; each
    policy reads only the fields it needs. This keeps per-policy construction
    knowledge in the policies, not in the run.
    """

    known_path: list
    start_xy: tuple = (0.0, 0.0)
    accepted_runs: list = field(default_factory=list)
    known_blockers: list = field(default_factory=list)
    known_wall_segments: list = field(default_factory=list)
    territory_mm: float = 0.0
    sample_distances: tuple = ()


class ExplorationPolicy(ABC):
    """Pluggable source of positive heading preference for the explorer.

    Defaults describe an *unconstrained* policy: the whole plane is allowed,
    forward legs are never clamped, nothing unlocks or expands, and no progress
    is reported. Subclasses that add a constraint (e.g. bounded territories)
    override the relevant hooks.
    """

    #: key under which this policy's per-run metadata is stored in the map JSON.
    metadata_key = 'exploration_policy'

    @classmethod
    def from_context(cls, context):
        """Build the policy from a run's :class:`PolicyContext`.

        Default: construct with no arguments. Policies that need run state (the
        live path, territory geometry, prior runs) override this and read the
        fields they want off ``context``.
        """
        return cls()

    def drive(self, run, duration):
        """Drive ``run`` for ``duration`` seconds, then return.

        Default driver for a heading-preference policy: orient toward known
        landmarks once, then run the run's argmax exploration loop (which scores
        candidate headings with ``heading_preference``). Self-driving policies
        (e.g. the wall follower) override this with their own loop.
        """
        run.turn_toward_knowledge('initial strategy')
        run.explore_until(time.time() + duration)

    def blocked_territory_crossing(self, previous_xy, current_xy):
        """For a bounded policy, the ``(from_cell, to_cell)`` of a blocked leg
        that crossed into another territory (which the run then reverts), or
        ``None``. Unbounded policies never reject -- the default is ``None``.
        """
        return None

    def revisit_crosses_territory(self, current_xy, corrected_xy):
        """Whether a revisit/loop-closure correction would jump territories.

        A bounded policy forbids such a correction; an unbounded one never does
        (default ``False``).
        """
        return False

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
