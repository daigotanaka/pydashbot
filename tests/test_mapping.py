import json
import unittest
from datetime import datetime
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from examples.mapping import calibrate
from examples.mapping import conservative_exploration as conservative
from examples.mapping import exploration_policies
from examples.mapping import map_room

FIXTURES = Path(__file__).parent / "data"


class MappingStrategyTests(unittest.TestCase):
    def test_output_paths_include_requested_timestamp(self):
        now = datetime(2026, 6, 12, 14, 5, 9)
        expected = Path("calibration_20260612-14-05-09.json")
        self.assertEqual(calibrate.timestamped_path("calibration", ".json", now), expected)
        self.assertEqual(map_room.timestamped_path("calibration", ".json", now), expected)
        self.assertEqual(
            map_room.timestamped_path("room_map", ".png", now),
            Path("room_map_20260612-14-05-09.png"),
        )

    def test_calibration_output_argument_accepts_exact_file_path(self):
        self.assertEqual(
            calibrate.parse_args(["--output", "data/my_calibration.json"]).output,
            "data/my_calibration.json",
        )

    def test_mapping_config_requires_positive_duration(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text('{"map_file":"map.json","duration_seconds":0}')
            with self.assertRaises(SystemExit):
                map_room.parse_args(["start", "--config", str(config)])

    def test_default_mapping_config_uses_conservative_exploration(self):
        self.assertFalse(map_room.parse_args(["start"]).no_conservative_exploration)

    def test_mapping_config_sets_reusable_run_options(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text(
                json.dumps(
                    {
                        "map_file": "data/room_map.json",
                        "calibration": "data/calibration.json",
                        "duration_seconds": 300,
                        "conservative_exploration": False,
                        "territory_size_mm": 1500,
                        "go_home_strategy": "hard-blocked-edge",
                    }
                )
            )

            options = map_room.parse_args(["dock", "--config", str(config)])

        self.assertEqual(options.mode, "dock")
        self.assertEqual(options.map_file, "data/room_map.json")
        self.assertEqual(options.calibration, "data/calibration.json")
        self.assertEqual(options.duration, 300)
        self.assertTrue(options.no_conservative_exploration)
        self.assertEqual(options.territory_size, 1500)
        self.assertEqual(options.go_home_strategy, "hard-blocked-edge")

    def test_mapping_config_preserves_policy_priority_order(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text(
                "map_file: data/room_map.json\n"
                "policy:\n"
                "  - name: preset\n"
                "    input_file: data/course.json\n"
                "  - name: later-policy\n"
            )

            options = map_room.parse_args(["start", "--config", str(config)])

        self.assertEqual(
            options.policy,
            [
                {"name": "preset", "input_file": "data/course.json"},
                {"name": "later-policy"},
            ],
        )

    def test_custom_config_file_is_selected_from_cli(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text(
                json.dumps(
                    {
                        "map_file": "new.json",
                        "duration_seconds": 300,
                        "go_home_strategy": "hard-blocked-edge",
                    }
                )
            )

            options = map_room.parse_args(["resume", "--config", str(config)])

        self.assertEqual(options.mode, "resume")
        self.assertEqual(options.map_file, "new.json")
        self.assertEqual(options.duration, 300)
        self.assertEqual(options.go_home_strategy, "hard-blocked-edge")

    def test_mapping_config_rejects_unknown_settings(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text('{"mystery": true}')
            with self.assertRaises(SystemExit):
                map_room.parse_args(["start", "--config", str(config)])

    def test_mapping_config_accepts_yaml_comments(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text(
                "map_file: room_map.json\n"
                "# Available strategies: d-star-lite, hard-blocked-edge\n"
                "go_home_strategy: hard-blocked-edge\n"
            )

            options = map_room.parse_args(["dock", "--config", str(config)])

        self.assertEqual(options.map_file, "room_map.json")
        self.assertEqual(options.go_home_strategy, "hard-blocked-edge")

    def test_mapping_config_rejects_invalid_yaml(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text("map_file: [\n")
            with self.assertRaises(SystemExit):
                map_room.parse_args(["start", "--config", str(config)])

    def test_mapping_config_requires_map_file(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text('{"duration_seconds": 60}')
            with self.assertRaises(SystemExit):
                map_room.parse_args(["start", "--config", str(config)])

    def test_latest_calibration_file_uses_timestamped_name(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            older = root / "calibration_20260611-23-59-59.json"
            newer = root / "calibration_20260612-14-05-09.json"
            legacy = root / "calibration.json"
            for path in (newer, legacy, older):
                path.write_text("{}")
            self.assertEqual(map_room.latest_calibration_file(root), newer)

    def test_mapping_run_mode_is_positional(self):
        self.assertEqual(map_room.parse_args(["start"]).mode, "start")
        self.assertEqual(map_room.parse_args(["resume"]).mode, "resume")
        self.assertEqual(map_room.parse_args(["dock"]).mode, "dock")
        with self.assertRaises(SystemExit):
            map_room.parse_args(["go_home"])

    def test_resume_and_dock_require_configured_map_before_robot_commands(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text(
                json.dumps({"map_file": str(Path(directory) / "missing.json")})
            )
            for mode in ("resume", "dock"):
                with (
                    self.subTest(mode=mode),
                    patch.object(map_room, "send_command") as send_command,
                    self.assertRaisesRegex(ValueError, "requires existing map file"),
                ):
                    map_room.main([mode, "--config", str(config)])
                send_command.assert_not_called()

    def test_load_resume_state_uses_last_pose_and_map_calibration(self):
        with TemporaryDirectory() as directory:
            map_file = Path(directory) / "room_map.json"
            map_file.write_text(
                '{"calibration":{"deg_per_yaw":0.5,"mm_per_wd":0.25},'
                '"runs":[{"path":[[1,2,3],[4,5,6]]}]}'
            )
            self.assertEqual(
                map_room.load_resume_state(map_file),
                (0.5, 0.25, 4.0, 5.0, 6.0),
            )

    def test_map_start_pose_uses_first_saved_pose(self):
        data = {"runs": [{"path": [[10, 20, 30], [40, 50, 60]]}]}
        self.assertEqual(map_room.map_start_pose(data), (10.0, 20.0, 30.0))

    def test_map_knowledge_densifies_each_run_without_connecting_sessions(self):
        data = {
            "runs": [
                {"path": [[0, 0, 0], [100, 0, 0]]},
                {"path": [[900, 0, 0], [1000, 0, 0]]},
            ]
        }

        path, _ = map_room.map_knowledge(data, path_sample_mm=50)

        self.assertIn((50.0, 0.0), path)
        self.assertIn((950.0, 0.0), path)
        self.assertNotIn((500.0, 0.0), path)

    def test_preset_course_visits_expected_cells_and_ends_in_cell_zero_two(self):
        policy = exploration_policies.load_exploration_policy(
            [
                {"name": "preset", "input_file": str(FIXTURES / "course.json")},
                {"name": "lower-priority-policy"},
            ]
        )
        readings = {
            "get_yaw": iter([0]),
            "get_left_wheel": iter([0]),
            "get_right_wheel": iter([0]),
            "get_pitch": iter([0] * 8),
            "get_prox_left": iter([0] * 3),
            "get_prox_right": iter([0] * 3),
        }
        calls = []

        def send_command(method, *args, **kwargs):
            calls.append((method, args, kwargs))
            if method in readings:
                return {"ok": True, "result": next(readings[method])}
            return {"ok": True, "result": None}

        with (
            patch.object(map_room, "send_command", side_effect=send_command),
            patch.object(
                map_room.time,
                "time",
                side_effect=AssertionError(
                    "preset policy must not wait for or inspect the timeout"
                ),
            ),
            patch.object(
                map_room,
                "read_settled",
                side_effect=[
                    0, 250, 250,
                    -90, 300, 200,
                    -90, 550, 450,
                    -180, 600, 400,
                    -180, 850, 650,
                ],
            ),
        ):
            run = map_room.explore(
                1.0,
                1.0,
                80.0,
                -80.0,
                duration=60,
                command_policy=policy,
            )

        cells = [
            conservative.local_grid_cell((0, -1), point[0], point[1])
            for point in run["path"]
        ]
        self.assertEqual(
            cells,
            [(0, 3), (1, 3), (1, 3), (1, 2), (1, 2), (0, 2)],
        )
        self.assertAlmostEqual(run["path"][-1][0], 80.0)
        self.assertAlmostEqual(run["path"][-1][1], -330.0)
        resolution = conservative.territory_resolution(
            (0, -1),
            conservative.densify_path(run["path"], 125),
            [],
        )
        self.assertEqual(
            resolution["visited"],
            {(0, 2), (0, 3), (1, 2), (1, 3)},
        )
        self.assertEqual(run["exploration_policy"]["name"], "preset")
        self.assertTrue(run["exploration_policy"]["completed"])
        self.assertEqual(run["exploration_policy"]["commands_completed"], 5)
        moves = [call for call in calls if call[0] == "move"]
        self.assertEqual([move[1][0] for move in moves], [250, 250, 250])
        self.assertEqual([move[1][1] for move in moves], [200, 200, 200])
        self.assertFalse(moves[0][2]["stop_at_obstacle"])
        self.assertTrue(moves[1][2]["stop_at_obstacle"])
        self.assertTrue(moves[2][2]["stop_at_obstacle"])

    def test_home_route_uses_shortest_connected_traversed_path(self):
        data = {
            "runs": [
                {
                    "status": "accepted",
                    "path": [
                        [0, 0, 0],
                        [1000, 0, 0],
                        [1000, 1000, 90],
                        [100, 100, 180],
                    ],
                }
            ]
        }
        self.assertEqual(
            map_room.plan_home_route(data),
            [(100.0, 100.0), (0.0, 0.0)],
        )

    def test_home_route_follows_proven_segments_without_unknown_shortcut(self):
        data = {
            "runs": [
                {
                    "status": "accepted",
                    "path": [
                        [0, 0, 0],
                        [1000, 0, 0],
                        [1000, 1000, 90],
                    ],
                }
            ]
        }
        self.assertEqual(
            map_room.plan_home_route(data),
            [(1000.0, 1000.0), (1000.0, 0.0), (0.0, 0.0)],
        )

    def test_home_route_removes_wall_contact_backtracking_spike(self):
        self.assertEqual(
            map_room.simplify_home_route(
                [(800, 0), (1000, 0), (0, 0)]
            ),
            [(800.0, 0.0), (0.0, 0.0)],
        )

    def test_home_route_rejects_untrustworthy_final_pose(self):
        data = {
            "runs": [
                {
                    "status": "partial",
                    "quality": {"tracking_lost": True},
                    "path": [[0, 0, 0], [1000, 0, 0]],
                }
            ]
        }
        with self.assertRaisesRegex(ValueError, "trustworthy final saved pose"):
            map_room.plan_home_route(data)

    def test_home_route_accepts_safely_aborted_go_home_pose(self):
        data = {
            "runs": [
                {
                    "status": "accepted",
                    "path": [[0, 0, 0], [1000, 0, 0]],
                },
                {
                    "mode": "go_home",
                    "status": "partial",
                    "quality": {
                        "tracking_lost": True,
                        "rejected_updates": 0,
                        "issues": ["go-home leg stopped early"],
                    },
                    "path": [[1000, 0, 0]],
                },
            ]
        }
        self.assertEqual(
            map_room.plan_home_route(data),
            [(1000.0, 0.0), (0.0, 0.0)],
        )

    def test_collect_blocked_edges_reads_recorded_segments(self):
        data = {
            "runs": [
                {"path": [[0, 0, 0]]},
                {"blocked_edges": [{"from": [1, 2], "to": [3, 4], "stop": [2, 3]}]},
            ]
        }
        self.assertEqual(
            map_room.collect_blocked_edges(data),
            [((1.0, 2.0), (3.0, 4.0))],
        )

    def test_edge_is_blocked_excludes_only_collinear_overlap(self):
        blocked = [((0, 0), (1000, 0))]
        self.assertTrue(map_room.edge_is_blocked((200, 0), (800, 0), blocked))
        # A corridor crossing the blocked segment perpendicularly is still usable.
        self.assertFalse(map_room.edge_is_blocked((500, -300), (500, 300), blocked))

    def test_blocked_edge_does_not_over_block_parallel_proven_corridor(self):
        # Real recorded case: a go-home block on (532,25)->(738,69) must not
        # exclude the proven corridor (80,80)->(738,69) that shares an endpoint
        # and runs past the blocked segment's far end.
        blocked = [((532, 25), (738, 69))]
        self.assertFalse(
            map_room.edge_is_blocked((80, 80), (738, 69), blocked)
        )
        # The blocked approach itself is still excluded.
        self.assertTrue(
            map_room.edge_is_blocked((560, 30), (720, 65), blocked)
        )

    def test_home_route_avoids_blocked_segment(self):
        data = {
            "runs": [
                {"status": "accepted", "path": [[0, 0, 0], [1000, 0, 0], [1000, 1000, 0]]},
                {"status": "accepted", "path": [[0, 0, 0], [0, 1000, 0], [1000, 1000, 0]]},
                {
                    "mode": "go_home",
                    "status": "partial",
                    "quality": {
                        "tracking_lost": False,
                        "rejected_updates": 0,
                        "issues": ["go-home leg stopped early"],
                    },
                    "path": [[1000, 1000, 0]],
                    "blocked_edges": [
                        {"from": [1000, 1000], "to": [1000, 0], "stop": [1000, 800]}
                    ],
                },
            ]
        }
        route = map_room.plan_home_route(data)
        # The eastern corridor is blocked, so it must detour via the north.
        self.assertIn((0.0, 1000.0), route)
        self.assertNotIn((1000.0, 0.0), route)

    def test_home_route_reports_when_all_routes_blocked(self):
        data = {
            "runs": [
                {"status": "accepted", "path": [[0, 0, 0], [1000, 0, 0]]},
                {
                    "mode": "go_home",
                    "status": "partial",
                    "quality": {
                        "tracking_lost": False,
                        "rejected_updates": 0,
                        "issues": ["go-home leg stopped early"],
                    },
                    "path": [[1000, 0, 0]],
                    "blocked_edges": [
                        {"from": [1000, 0], "to": [0, 0], "stop": [600, 0]}
                    ],
                },
            ]
        }
        with self.assertRaisesRegex(ValueError, "no unblocked proven route"):
            map_room.plan_home_route(data, map_room.LEGACY_GO_HOME_STRATEGY)

    def test_d_star_lite_softens_blocked_route_instead_of_deleting_it(self):
        data = {
            "runs": [
                {"status": "accepted", "path": [[0, 0, 0], [1000, 0, 0]]},
                {
                    "mode": "go_home",
                    "status": "partial",
                    "quality": {
                        "tracking_lost": False,
                        "rejected_updates": 0,
                        "issues": ["go-home leg stopped early"],
                    },
                    "path": [[1000, 0, 0]],
                    "blocked_edges": [
                        {"from": [1000, 0], "to": [0, 0], "stop": [600, 0]}
                    ],
                },
            ]
        }

        self.assertEqual(
            map_room.plan_home_route(data),
            [(1000.0, 0.0), (0.0, 0.0)],
        )

    def test_active_go_home_strategy_is_d_star_lite(self):
        self.assertEqual(map_room.ACTIVE_GO_HOME_STRATEGY.name, "d-star-lite")

    def test_obstacle_near_home_counts_as_arrival(self):
        obstacle = {"halt": "obstacle", "side": "front", "prox_left": 30, "prox_right": 27}
        # Obstacles within tolerance of home are arrival at the corner walls.
        self.assertTrue(map_room.obstacle_arrival_near_home(231, obstacle))
        self.assertTrue(map_room.obstacle_arrival_near_home(327, obstacle))
        # The same obstacle far from home is a genuine blocked route.
        self.assertFalse(map_room.obstacle_arrival_near_home(800, obstacle))
        # A non-obstacle halt near home does not count (e.g. odometry/turn issue).
        self.assertFalse(map_room.obstacle_arrival_near_home(100, {"halt": "completed"}))
        self.assertFalse(map_room.obstacle_arrival_near_home(100, None))

    def test_wall_clearance_only_after_large_turn_into_a_wall(self):
        threshold = map_room.PROX_THRESHOLD
        # Near-reversal turn facing a wall just left -> nudge to clear it.
        self.assertTrue(map_room.needs_wall_clearance(180, 0, threshold))
        self.assertTrue(map_room.needs_wall_clearance(-150, threshold + 5, 0))
        # A large turn but a clear front does not qualify.
        self.assertFalse(map_room.needs_wall_clearance(180, 0, threshold - 1))
        # A small heading change never triggers a clearance nudge.
        self.assertFalse(map_room.needs_wall_clearance(30, threshold, threshold))
        # Missing readings are treated as clear.
        self.assertFalse(map_room.needs_wall_clearance(180, None, None))

    def test_go_home_returns_to_initial_pose_and_heading(self):
        data = {
            "runs": [
                {
                    "status": "accepted",
                    "path": [[0, 0, 0], [1000, 0, 90]],
                }
            ]
        }
        readings = {
            "get_yaw": iter([0]),
            "get_left_wheel": iter([0]),
            "get_right_wheel": iter([0]),
        }
        calls = []

        def send_command(method, *args, **kwargs):
            calls.append((method, args, kwargs))
            if method in readings:
                return {"result": next(readings[method])}
            return {"result": None}

        with (
            patch.object(map_room, "send_command", side_effect=send_command),
            patch.object(
                map_room,
                "read_settled",
                side_effect=[
                    90, -100, 100,
                    90, 900, 1100,
                    0, 1100, 900,
                    -90, 1300, 700,
                ],
            ),
        ):
            run = map_room.go_home(data, 1.0, 1.0)

        self.assertEqual(run["status"], "accepted")
        self.assertEqual(run["mode"], "go_home")
        self.assertAlmostEqual(run["path"][-1][0], 0)
        self.assertAlmostEqual(run["path"][-1][1], 0)
        self.assertAlmostEqual(run["path"][-1][2], 0)
        self.assertIn(
            (
                "move",
                (1000, map_room.FORWARD_SPEED_MMPS),
                {
                    "wall_stop_sound": None,
                    "proximity_threshold": map_room.HOME_RETRACE_PROX_THRESHOLD,
                    "proximity_confirm_count": map_room.HOME_RETRACE_CONFIRM_COUNT,
                },
            ),
            calls,
        )

    def _aborted_home_run(self, blocked=True):
        return {
            "mode": "go_home",
            "status": "partial",
            "quality": {
                "tracking_lost": False,
                "rejected_updates": 0,
                "issues": ["go-home leg stopped early"],
            },
            "blocked_edges": [{"from": [1, 1], "to": [2, 2], "stop": [1, 1]}]
            if blocked
            else [],
        }

    def _accepted_home_run(self):
        return {
            "mode": "go_home",
            "status": "accepted",
            "quality": {"tracking_lost": False},
            "blocked_edges": [],
        }

    def test_go_home_retries_replan_until_arrival(self):
        sequence = [
            self._aborted_home_run(),
            self._aborted_home_run(),
            self._accepted_home_run(),
        ]
        with patch.object(map_room, "go_home", side_effect=sequence) as go_home:
            runs = map_room.go_home_with_retries({"runs": []}, 1.0, 1.0, max_retries=3)
        self.assertEqual(go_home.call_count, 3)
        self.assertEqual(runs[-1]["status"], "accepted")

    def test_go_home_retries_respect_the_max(self):
        sequence = [self._aborted_home_run() for _ in range(10)]
        with patch.object(map_room, "go_home", side_effect=sequence) as go_home:
            runs = map_room.go_home_with_retries({"runs": []}, 1.0, 1.0, max_retries=3)
        # One initial attempt plus three retries.
        self.assertEqual(go_home.call_count, 4)
        self.assertEqual(len(runs), 4)

    def test_go_home_stops_when_halt_is_not_a_blockage(self):
        with patch.object(
            map_room, "go_home", side_effect=[self._aborted_home_run(blocked=False)]
        ) as go_home:
            map_room.go_home_with_retries({"runs": []}, 1.0, 1.0, max_retries=3)
        self.assertEqual(go_home.call_count, 1)

    def test_go_home_stops_when_no_route_remains(self):
        with patch.object(
            map_room, "go_home", side_effect=ValueError("no unblocked route")
        ) as go_home:
            runs = map_room.go_home_with_retries({"runs": []}, 1.0, 1.0, max_retries=3)
        self.assertEqual(go_home.call_count, 1)
        self.assertEqual(runs, [])

    def test_latest_map_file_includes_legacy_map(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            legacy = root / "room_map.json"
            legacy.write_text("{}")
            self.assertEqual(map_room.latest_map_file(root), legacy)

    def test_forward_distance_is_integer_and_fits_remaining_time(self):
        self.assertEqual(map_room.forward_distance_for_remaining(60), 3000)
        self.assertEqual(map_room.forward_distance_for_remaining(2.5), 500)
        self.assertEqual(map_room.forward_distance_for_remaining(0.1), 200)
        self.assertEqual(map_room.home_leg_distance(452.429), 452)
        self.assertEqual(map_room.home_leg_distance(1200.5), 1000)

    def test_corner_dock_turns_left_toward_side_wall_then_right_into_room(self):
        calls = []
        readings = {
            "get_prox_rear": iter([map_room.REAR_THRESHOLD]),
            "get_prox_left": iter([map_room.PROX_THRESHOLD]),
            "get_prox_right": iter([0]),
        }

        def send_command(method, *args, **kwargs):
            calls.append((method, args, kwargs))
            if method in readings:
                return {"result": next(readings[method])}
            return {"result": None}

        with (
            patch.object(map_room, "send_command", side_effect=send_command),
            patch.object(map_room.time, "sleep"),
        ):
            pose = map_room.dock_to_corner(1.0, 1.0)

        turns = [args[0] for method, args, _ in calls if method == "turn"]
        self.assertEqual(turns, [90, -90])
        self.assertEqual(pose, (80.0, -80.0))

    def test_odometry_validation_rejects_negative_forward_distance(self):
        issues = map_room.validate_odometry("forward", 3000, -1668, -2.6)
        self.assertIn("forward move measured negative distance", issues)

    def test_wheel_distance_wrap_across_sign_boundary_stays_forward(self):
        self.assertEqual(
            map_room.wrap_delta(0x7FFF0, 0x80020, 20),
            0x30,
        )

    def test_yaw_wrap_reports_short_positive_turn(self):
        self.assertEqual(map_room.wrap_delta(1500, -1314, 12), 1282)
        self.assertEqual(map_room.wrap_delta(0, 55574, 12), -1770)

    def test_large_tracked_turns_are_split_into_unambiguous_steps(self):
        self.assertEqual(map_room.tracked_turn_steps(90), [90])
        self.assertEqual(map_room.tracked_turn_steps(-150), [-75, -75])
        self.assertEqual(map_room.tracked_turn_steps(180), [90, 90])

    def test_wheel_translation_averages_both_wheels(self):
        self.assertEqual(
            map_room.wheel_translation_delta(100, 600, 200, 700),
            500,
        )
        self.assertEqual(
            map_room.wheel_translation_delta(500, 200, 200, 500),
            0,
        )

    def test_odometry_validation_rejects_implausible_turn(self):
        issues = map_room.validate_odometry("turn", 90, 0, 372)
        self.assertIn("turn measured excessive heading change", issues)

    def test_odometry_validation_rejects_forward_wheel_slip(self):
        # Captured slip: right wheel spun 2.16x the left (implying a ~208 deg
        # turn) while the gyro measured only 4.6 deg on a straight move.
        issues = map_room.validate_odometry(
            "forward", 425, 429.1, 4.6,
            left_delta=1366, right_delta=2954, mm_per_wheel_tick=0.1987,
        )
        self.assertIn(
            "wheel rotation inconsistent with gyro (suspected wheel slip)",
            issues,
        )

    def test_odometry_validation_accepts_clean_forward_wheels(self):
        issues = map_room.validate_odometry(
            "forward", 1000, 1009.5, -0.14,
            left_delta=5082, right_delta=5080, mm_per_wheel_tick=0.1987,
        )
        self.assertEqual(issues, [])

    def test_odometry_slip_check_excludes_turns(self):
        # A turn where the wheels imply far more rotation than the gyro saw is
        # not a pose-corrupting slip (heading comes from the gyro, translation
        # is ~0), so it must not be flagged.
        issues = map_room.validate_odometry(
            "turn", 47, 1.7, 47.4,
            left_delta=-699, right_delta=716, mm_per_wheel_tick=0.1987,
        )
        self.assertNotIn(
            "wheel rotation inconsistent with gyro (suspected wheel slip)",
            issues,
        )

    def test_odometry_validation_without_wheel_data_skips_slip_check(self):
        self.assertEqual(map_room.validate_odometry("forward", 100, 105, 1.0), [])

    def test_map_knowledge_uses_only_accepted_runs(self):
        data = {
            "schema_version": 2,
            "runs": [
                {
                    "status": "accepted",
                    "path": [[1, 2, 0]],
                    "walls": [[3, 4]],
                    "obstacles": [],
                },
                {
                    "status": "suspect",
                    "path": [[100, 200, 0]],
                    "walls": [[300, 400]],
                    "obstacles": [],
                },
            ],
        }
        self.assertEqual(map_room.map_knowledge(data), ([(1.0, 2.0)], [(3.0, 4.0)]))

    def test_revisit_pose_correction_moves_toward_known_landmark(self):
        correction = map_room.revisit_pose_correction(
            100,
            100,
            (250, 100),
            [(0, 0), (110, 90)],
            [(150, 100)],
        )
        dx, dy, target, mismatch = correction
        self.assertEqual(target, (150, 100))
        self.assertEqual(mismatch, 100)
        self.assertEqual(dx, -60)
        self.assertEqual(dy, 0)

    def test_revisit_pose_correction_ignores_unrecognized_area(self):
        self.assertIsNone(
            map_room.revisit_pose_correction(
                5000,
                5000,
                (5150, 5000),
                [(0, 0)],
                [(150, 0)],
            )
        )
        self.assertIsNone(
            map_room.revisit_pose_correction(
                100,
                100,
                (250, 100),
                [(100, 100)],
                [(1000, 1000)],
            )
        )

    def test_strategy_avoids_known_blockers(self):
        blockers = [(400, 0), (800, 0), (1200, 0)]
        angle = map_room.choose_exploration_angle(
            0, 0, 0, [(0, 0)], blockers
        )
        self.assertNotEqual(angle, 0)

    def test_core_strategy_avoids_inferred_continuous_wall(self):
        walls = [(400, -100), (400, 100)]
        wall_segments = map_room.inferred_wall_segments(walls)
        angle = map_room.choose_exploration_angle(
            0,
            0,
            0,
            [(0, 0)],
            [],
            wall_segments=wall_segments,
        )
        self.assertNotEqual(angle, 0)

    def test_strategy_turns_away_from_live_proximity(self):
        left_blocked = map_room.choose_exploration_angle(
            0, 0, 0, [], [], blocked_left=20, require_turn=True
        )
        right_blocked = map_room.choose_exploration_angle(
            0, 0, 0, [], [], blocked_right=20, require_turn=True
        )
        self.assertLess(left_blocked, 0)
        self.assertGreater(right_blocked, 0)

    def test_strategy_rewards_unexplored_direction(self):
        explored_east = [(distance, 0) for distance in (400, 800, 1200, 1600)]
        angle = map_room.choose_exploration_angle(
            0, 0, 0, explored_east, [], require_turn=False
        )
        self.assertNotEqual(angle, 0)

    def test_conservative_strategy_turns_before_mental_wall(self):
        angle = map_room.choose_exploration_angle(
            1800,
            1000,
            0,
            [(1800, 1000)],
            [],
            point_allowed=lambda x, y: conservative.territory_cell(x, y) == (0, 0),
        )
        self.assertNotEqual(angle, 0)

    def test_conservative_state_is_metadata_not_real_wall_knowledge(self):
        data = {
            "runs": [
                {
                    "status": "accepted",
                    "path": [[80, 80, 0]],
                    "walls": [],
                    "obstacles": [],
                    "conservative_exploration": {
                        "territories": [[0, 0], [1, 0]],
                        "focus_territory": [1, 0],
                    },
                }
            ]
        }
        policy = conservative.ConservativeExploration(
            map_room.accepted_runs(data),
            (80, 80),
            *map_room.map_knowledge(data),
        )
        self.assertEqual((policy.territories, policy.focus), ([(0, 0), (1, 0)], (1, 0)))
        self.assertEqual(map_room.map_knowledge(data), ([(80.0, 80.0)], []))

    def test_explore_turns_at_mental_wall_without_recording_real_wall(self):
        readings = {
            "get_yaw": iter([0]),
            "get_left_wheel": iter([0]),
            "get_right_wheel": iter([0]),
            "get_pitch": iter([0, 0, 0, 0, 0, 0]),
            "get_prox_left": iter([0]),
            "get_prox_right": iter([0]),
        }
        calls = []

        def send_command(method, *args, **kwargs):
            calls.append((method, args, kwargs))
            if method in readings:
                return {"result": next(readings[method])}
            return {"result": None}

        times = iter([0.0, 0.0, 0.0, 1.0, 1.0, 4.0])
        with (
            patch.object(map_room, "send_command", side_effect=send_command),
            patch.object(map_room.time, "time", side_effect=lambda: next(times)),
            patch("builtins.print") as print_mock,
            patch.object(
                map_room,
                "choose_exploration_angle",
                side_effect=[0, 90],
            ),
            patch.object(
                map_room,
                "read_settled",
                side_effect=[
                    0, 325, 325,
                    100, 325, 325,
                ],
            ),
        ):
            run = map_room.explore(
                1.0, 1.0, 1500.0, 1000.0, duration=3, territory_mm=2000
            )

        move = next(call for call in calls if call[0] == "move")
        self.assertEqual(move[1], (325, map_room.FORWARD_SPEED_MMPS))
        self.assertEqual(run["walls"], [])
        self.assertEqual(run["obstacles"], [])
        self.assertEqual(run["conservative_exploration"]["territories"], [(0, 0)])
        output = " ".join(str(call) for call in print_mock.call_args_list)
        self.assertIn("[cell complete]", output)

    def test_explore_can_run_without_conservative_policy(self):
        calls = []
        readings = {
            "get_yaw": iter([0]),
            "get_left_wheel": iter([0]),
            "get_right_wheel": iter([0]),
            "get_pitch": iter([0, 0, 0, 0, 0, 0]),
            "get_prox_left": iter([0]),
            "get_prox_right": iter([0]),
        }

        def send_command(method, *args, **kwargs):
            calls.append((method, args, kwargs))
            if method in readings:
                return {"result": next(readings[method])}
            return {"result": None}

        times = iter([0.0, 0.0, 0.0, 1.0, 1.0, 4.0])
        with (
            patch.object(map_room, "send_command", side_effect=send_command),
            patch.object(map_room.time, "time", side_effect=lambda: next(times)),
            patch.object(map_room, "choose_exploration_angle", side_effect=[0, 90]),
            patch.object(
                map_room,
                "read_settled",
                side_effect=[0, 600, 600, 100, 600, 600],
            ),
        ):
            run = map_room.explore(
                1.0,
                1.0,
                1500.0,
                1000.0,
                duration=3,
                conservative_exploration=False,
            )

        move = next(call for call in calls if call[0] == "move")
        self.assertEqual(move[1], (600, map_room.FORWARD_SPEED_MMPS))
        self.assertNotIn("conservative_exploration", run)

    def test_explore_announces_adjacent_territory_unlock(self):
        visited = [[100, y, 90] for y in (100, 600, 1100, 1600)]
        barrier = [[600, y] for y in (100, 600, 1100, 1600)]
        strategy_map = {
            "schema_version": 2,
            "runs": [
                {
                    "status": "accepted",
                    "path": visited,
                    "walls": barrier,
                    "obstacles": [],
                    "conservative_exploration": {
                        "territories": [[0, 0]],
                        "focus_territory": [0, 0],
                    },
                }
            ],
        }
        readings = {
            "get_yaw": iter([0]),
            "get_left_wheel": iter([0]),
            "get_right_wheel": iter([0]),
            "get_pitch": iter([0, 0, 0, 0, 0, 0]),
            "get_prox_left": iter([0]),
            "get_prox_right": iter([0]),
        }

        def send_command(method, *args, **kwargs):
            if method in readings:
                return {"result": next(readings[method])}
            return {"result": None}

        times = iter([0.0, 0.0, 0.0, 0.5])
        with (
            patch.object(map_room, "send_command", side_effect=send_command),
            patch.object(map_room.time, "time", side_effect=lambda: next(times)),
            patch.object(map_room, "choose_exploration_angle", return_value=0),
            patch.object(map_room, "read_settled", side_effect=[0, -1000, -1000]),
            patch("builtins.print") as print_mock,
        ):
            run = map_room.explore(
                1.0,
                1.0,
                100.0,
                100.0,
                strategy_map=strategy_map,
                duration=1,
                territory_mm=2000,
            )

        output = " ".join(str(call) for call in print_mock.call_args_list)
        self.assertIn("[adjacent territory unlocked]", output)
        self.assertEqual(
            run["conservative_exploration"]["territories"],
            [(0, 0), (1, 0)],
        )

    def test_explore_moves_until_wall_then_turns_and_continues(self):
        calls = []
        readings = {
            "get_yaw": iter([0]),
            "get_left_wheel": iter([0]),
            "get_right_wheel": iter([0]),
            "get_pitch": iter([0, 0, 0, 0, 0, 0]),
            "get_prox_left": iter([20]),
            "get_prox_right": iter([0]),
        }

        def send_command(method, *args, **kwargs):
            calls.append((method, args, kwargs))
            if method in readings:
                return {"result": next(readings[method])}
            return {"result": None}

        times = iter([0.0, 0.0, 0.0, 0.5, 0.5, 1.1])
        with (
            patch.object(map_room, "send_command", side_effect=send_command),
            patch.object(map_room.time, "time", side_effect=lambda: next(times)),
            patch.object(
                map_room,
                "choose_exploration_angle",
                side_effect=[0, 90],
            ),
            patch.object(map_room.random, "choice", return_value="okay"),
            patch.object(
                map_room,
                "read_settled",
                side_effect=[
                    0, 100, 100,
                    0, 50, 50,
                    100, 0, 100,
                ],
            ),
        ):
            run = map_room.explore(1.0, 1.0, 0.0, 0.0, duration=1)

        methods = [method for method, _, _ in calls]
        move = next(call for call in calls if call[0] == "move")
        self.assertEqual(move[1], (200, map_room.FORWARD_SPEED_MMPS))
        self.assertEqual(move[2], {"wall_stop_sound": None})
        self.assertIn("turn", methods)
        self.assertNotIn("drive", methods)
        self.assertEqual(run["status"], "accepted")
        self.assertEqual(len(run["path"]), 4)
        self.assertEqual(len(run["walls"]), 1)
        self.assertEqual(run["obstacles"], [])

    def test_rejected_odometry_stops_run_without_mapping_bad_pose(self):
        readings = {
            "get_yaw": iter([0]),
            "get_left_wheel": iter([0]),
            "get_right_wheel": iter([0]),
            "get_pitch": iter([0, 0, 0, 0, 0, 0]),
            "get_prox_left": iter([0]),
            "get_prox_right": iter([0]),
        }

        def send_command(method, *args, **kwargs):
            if method in readings:
                return {"result": next(readings[method])}
            return {"result": None}

        times = iter([0.0, 0.0, 0.0, 0.5])
        with (
            patch.object(map_room, "send_command", side_effect=send_command),
            patch.object(map_room.time, "time", side_effect=lambda: next(times)),
            patch.object(map_room, "choose_exploration_angle", return_value=0),
            patch.object(map_room, "read_settled", side_effect=[0, -1000, -1000]),
        ):
            run = map_room.explore(1.0, 1.0, 0.0, 0.0, duration=1)

        self.assertEqual(run["status"], "partial")
        self.assertEqual(run["path"], [(0.0, 0.0, 0.0)])
        self.assertEqual(run["walls"], [])
        self.assertEqual(run["quality"]["rejected_updates"], 1)
        self.assertFalse(run["events"][0]["accepted"])

    def test_save_map_excludes_suspect_run_observations(self):
        accepted = {
            "timestamp": "accepted",
            "status": "accepted",
            "path": [[0, 0, 0]],
            "walls": [[1, 2]],
            "obstacles": [],
            "events": [],
            "quality": {},
        }
        suspect = {
            "timestamp": "suspect",
            "status": "rejected",
            "path": [[9, 9, 0]],
            "walls": [[8, 8]],
            "obstacles": [[7, 7]],
            "events": [],
            "quality": {},
        }
        with TemporaryDirectory() as directory:
            map_file = Path(directory) / "room_map.json"
            map_room.save_map(1.0, 2.0, accepted, map_file)
            map_room.save_map(1.0, 2.0, suspect, map_file)
            data = json.loads(map_file.read_text())
        self.assertEqual(data["schema_version"], 2)
        self.assertEqual(data["walls"], [[1, 2]])
        self.assertEqual(data["obstacles"], [])

    def test_save_map_can_seed_separate_output_from_source_map(self):
        previous = {
            "schema_version": 2,
            "calibration": {"deg_per_yaw": 1.0, "mm_per_wd": 2.0},
            "runs": [
                {
                    "timestamp": "previous",
                    "status": "accepted",
                    "path": [[0, 0, 0]],
                    "walls": [[1, 2]],
                    "obstacles": [],
                    "events": [],
                    "quality": {},
                }
            ],
            "walls": [[1, 2]],
            "obstacles": [],
        }
        new_run = {
            "timestamp": "new",
            "status": "accepted",
            "path": [[3, 4, 0]],
            "walls": [[5, 6]],
            "obstacles": [],
            "events": [],
            "quality": {},
        }
        with TemporaryDirectory() as directory:
            output = Path(directory) / "nested" / "map.json"
            map_room.save_map(1.0, 2.0, new_run, output, base_data=previous)
            data = json.loads(output.read_text())
        self.assertEqual(len(data["runs"]), 2)
        self.assertEqual(data["walls"], [[1, 2], [5, 6]])

    def test_fresh_save_replaces_existing_map(self):
        old_run = {
            "timestamp": "old",
            "status": "accepted",
            "path": [[0, 0, 0]],
            "walls": [[1, 2]],
            "obstacles": [],
            "events": [],
            "quality": {},
        }
        new_run = {
            "timestamp": "new",
            "status": "accepted",
            "path": [[3, 4, 0]],
            "walls": [[5, 6]],
            "obstacles": [],
            "events": [],
            "quality": {},
        }
        with TemporaryDirectory() as directory:
            output = Path(directory) / "room_map.json"
            map_room.save_map(1.0, 2.0, old_run, output)
            map_room.save_map(
                1.0, 2.0, new_run, output, replace_existing=True
            )
            data = json.loads(output.read_text())
        self.assertEqual([run["timestamp"] for run in data["runs"]], ["new"])
        self.assertEqual(data["walls"], [[5, 6]])


if __name__ == "__main__":
    unittest.main()
