"""Default novelty exploration policy: undirected forward legs, no limit."""

import math

try:
    from apps.map.policies.exploration.exploration_policy_base import ExplorationPolicy
except ModuleNotFoundError:
    from policies.exploration.exploration_policy_base import ExplorationPolicy


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

    @classmethod
    def from_context(cls, context):
        return cls(context.known_path, context.sample_distances)

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
