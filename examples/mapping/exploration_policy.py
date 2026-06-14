"""Exploration-policy interface and the default novelty policy.

The turn decision is ``argmax`` over candidate headings of

    heading_score = policy.heading_preference  (positive / preferred)
                    - physical_constraints     (negative penalties)

``heading_score`` (in ``map_room``) owns the physical-constraint penalties.
All *positive* preference scoring lives in an ``ExplorationPolicy``. Concrete
policies override ``heading_preference``; every other hook has a safe,
unconstrained default so an unconstrained policy only implements the reward.

Each concrete policy lives in its own module (1 file, 1 class); the abstract
base and the default ``NoveltyExplorationPolicy`` share this one.
"""

import math
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


class NoveltyExplorationPolicy(ExplorationPolicy):
    """Default policy: prefer headings that lead into unexplored space.

    Rewards each sampled point ahead by its distance from the nearest prior
    path point (capped), which is the behaviour that used to be baked into
    ``heading_score`` as the novelty term. Imposes no territory constraint.
    """

    metadata_key = 'novelty_exploration'

    def __init__(self, path_points, sample_distances):
        # ``path_points`` is the live known-path list, so novelty reflects
        # everywhere the robot has been as exploration proceeds.
        self.path_points = path_points
        self.sample_distances = sample_distances

    def heading_preference(self, x, y, heading):
        hr = math.radians(heading)
        ux, uy = math.cos(hr), math.sin(hr)
        score = 0.0
        for distance in self.sample_distances:
            sx, sy = x + distance * ux, y + distance * uy
            if self.path_points:
                nearest = min(
                    math.hypot(sx - px, sy - py) for px, py in self.path_points
                )
                score += min(nearest, 800) * 0.35
            else:
                score += 280
        return score
