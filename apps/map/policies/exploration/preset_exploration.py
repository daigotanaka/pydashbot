"""Preset exploration policy: replay a fixed move/turn course from JSON.

Unlike the heading policies, which score candidate headings, the preset policy
emits an explicit sequence of move/turn commands the run drives verbatim -- so
like the wall follower it overrides ``drive`` instead of ``heading_preference``.
"""

import json
from pathlib import Path

try:
    from apps.map.policies.exploration.exploration_policy_base import ExplorationPolicy
except ModuleNotFoundError:
    from policies.exploration.exploration_policy_base import ExplorationPolicy


class PresetExplorationPolicy(ExplorationPolicy):
    """Replay a fixed sequence of move and turn commands from JSON."""

    name = 'preset'
    metadata_key = 'preset_exploration'

    def __init__(self, commands, input_file):
        self.commands = commands
        self.input_file = str(input_file)
        self.commands_completed = 0

    @classmethod
    def from_context(cls, context):
        return cls.from_input_file(context.exploration_options.get('input_file'))

    @classmethod
    def from_input_file(cls, input_file):
        if not isinstance(input_file, str) or not input_file.strip():
            raise ValueError("preset policy requires a non-empty input_file")
        path = Path(input_file)
        try:
            course = json.loads(path.read_text())
        except OSError as exc:
            raise ValueError(
                f"cannot read preset policy input_file {path}: {exc}"
            ) from exc
        except json.JSONDecodeError as exc:
            raise ValueError(
                f"invalid JSON in preset policy input_file {path}: {exc}"
            ) from exc

        commands = course.get('commands') if isinstance(course, dict) else course
        if not isinstance(commands, list) or not commands:
            raise ValueError(
                "preset policy input_file must contain a non-empty commands list"
            )
        return cls([validate_preset_command(command) for command in commands], path)

    def heading_preference(self, x, y, heading):
        return 0.0  # the course is explicit; no heading scoring is used

    def describe(self):
        return f"  Preset course: {len(self.commands)} fixed move/turn command(s)."

    def drive(self, run, duration):
        """Replay the course (ignoring the time budget) and record how far it got."""
        self.commands_completed = run.drive_preset_course(self.commands)

    def metadata(self):
        return {
            'name': self.name,
            'input_file': self.input_file,
            'commands': self.commands,
            'commands_completed': self.commands_completed,
            'completed': self.commands_completed == len(self.commands),
        }


def validate_preset_command(command):
    if (
        not isinstance(command, dict)
        or not {'command', 'value'} <= set(command)
        or set(command) - {'command', 'value', 'stop_at_obstacle'}
    ):
        raise ValueError(
            "each preset command must contain command and value; move commands "
            "may also specify stop_at_obstacle"
        )
    name = command['command']
    if name not in {'move', 'turn'}:
        raise ValueError(f"unsupported preset command: {name!r}")
    value = command['value']
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"preset {name} value must be numeric")
    if name == 'move' and value <= 0:
        raise ValueError("preset move value must be greater than zero")
    if name == 'turn' and not -180 <= value <= 180:
        raise ValueError("preset turn value must be between -180 and 180 degrees")
    stop_at_obstacle = command.get('stop_at_obstacle', True)
    if not isinstance(stop_at_obstacle, bool):
        raise ValueError("preset stop_at_obstacle must be true or false")
    if name == 'turn' and 'stop_at_obstacle' in command:
        raise ValueError("preset turn commands cannot specify stop_at_obstacle")
    validated = {
        'command': name,
        'value': int(round(value)) if name == 'move' else float(value),
    }
    if name == 'move':
        validated['stop_at_obstacle'] = stop_at_obstacle
    return validated
