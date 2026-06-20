import json
import unittest
from datetime import datetime
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from apps.map import calibrate
from apps.map.policies import conservative_exploration as conservative
from apps.map.policies import exploration_policies
from apps.map import main as map_room
from apps.map.policies.exploration_policy import NoveltyExplorationPolicy

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

    def test_mapping_config_defaults_to_conservative_exploration(self):
        # When the config omits exploration_policy, fall back to the default.
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text('{"map_file": "map.json"}')
            options = map_room.parse_args(["start", "--config", str(config)])
        self.assertEqual(options.exploration_policy, "conservative")
        self.assertEqual(
            options.docking,
            {"init": True, "go-home-strategy": "d-star-lite"},
        )
        self.assertEqual(options.go_home_strategy, "d-star-lite")
        self.assertEqual(
            options.dashboard,
            {"active": False, "host": "0.0.0.0", "port": 8000},
        )

    def test_mapping_config_accepts_dashboard_settings(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text(
                json.dumps(
                    {
                        "map_file": "map.json",
                        "dashboard": {
                            "active": True,
                            "host": "127.0.0.1",
                            "port": 9000,
                        },
                    }
                )
            )
            options = map_room.parse_args(["start", "--config", str(config)])
        self.assertEqual(
            options.dashboard,
            {"active": True, "host": "127.0.0.1", "port": 9000},
        )

    def test_dashboard_publisher_normalizes_bind_all_host(self):
        # 0.0.0.0 is a valid server bind address but not a connect target.
        publisher = map_room.DashboardPublisher("0.0.0.0", 8000)
        self.assertEqual(publisher.url, "http://127.0.0.1:8000/move")
        publisher = map_room.DashboardPublisher("192.168.1.5", 8000)
        self.assertEqual(publisher.url, "http://192.168.1.5:8000/move")

    def _dashboard_run(self):
        import types

        run = object.__new__(map_room._ExplorationRun)
        run.posts, run.amends = [], []
        run.dashboard = types.SimpleNamespace(
            post_move=run.posts.append, amend_move=run.amends.append
        )
        run._dashboard_warned = False
        run._dashboard_pending = False
        run.x = run.y = run.heading = 0.0
        run.walls, run.obstacles = [], []
        run._published_walls = run._published_obstacles = 0
        run.events = []
        return run

    def test_predict_posts_leg_target_before_moving(self):
        run = self._dashboard_run()
        # Forward 100 mm at heading 0 -> predicted end is +100 in x.
        run.predict_dashboard_pose("forward", 100)
        self.assertEqual(len(run.posts), 1)
        self.assertAlmostEqual(run.posts[-1]["pose"][0], 100.0, places=3)
        self.assertAlmostEqual(run.posts[-1]["duration"], 0.5, places=3)  # 100/200
        self.assertTrue(run._dashboard_pending)
        # A turn predicts the new heading, not a translation.
        run.predict_dashboard_pose("turn", 30)
        self.assertEqual(run.posts[-1]["pose"][:2], [0.0, 0.0])
        self.assertAlmostEqual(run.posts[-1]["pose"][2], map_room.normalize_heading(-30))

    def test_amend_sends_measured_pose_and_new_observations(self):
        run = self._dashboard_run()
        run.predict_dashboard_pose("forward", 750)

        # Robot stopped short at x=500 and observed a wall; amend reconciles it.
        run.x = 500.0
        run.events.append({"action": "forward", "distance_mm": 500})
        run.walls.append((625.0, 0.0))
        run.amend_dashboard_pose()
        self.assertEqual(len(run.amends), 1)
        self.assertEqual(run.amends[-1]["pose"][0], 500.0)
        self.assertEqual(run.amends[-1]["walls"], [[625.0, 0.0]])
        self.assertFalse(run._dashboard_pending)

        # No pending leg -> amend is a no-op (no double-send).
        run.amend_dashboard_pose()
        self.assertEqual(len(run.amends), 1)

        # A trailing observation after the last leg is flushed via amend.
        run.obstacles.append((10.0, 10.0))
        run.flush_dashboard_observations()
        self.assertEqual(len(run.amends), 2)
        self.assertEqual(run.amends[-1]["obstacles"], [[10.0, 10.0]])

    def test_mapping_config_accepts_wall_follower_policy(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text(
                '{"map_file":"map.json","exploration_policy":"wall-follower"}'
            )
            options = map_room.parse_args(["start", "--config", str(config)])
        self.assertEqual(options.exploration_policy, "wall-follower")

    def test_mapping_config_rejects_unknown_exploration_policy(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text(
                '{"map_file":"map.json","exploration_policy":"nope"}'
            )
            with self.assertRaises(SystemExit):
                map_room.parse_args(["start", "--config", str(config)])

    def test_mapping_config_sets_reusable_run_options(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text(
                json.dumps(
                    {
                        "map_file": "data/room_map.json",
                        "calibration": "data/calibration.json",
                        "duration_seconds": 300,
                        "exploration_policy": "coverage",
                        "territory_size_mm": 1500,
                        "docking": {
                            "init": False,
                            "go-home-strategy": "hard-blocked-edge",
                        },
                    }
                )
            )

            options = map_room.parse_args(["dock", "--config", str(config)])

        self.assertEqual(options.mode, "dock")
        self.assertEqual(options.map_file, "data/room_map.json")
        self.assertEqual(options.calibration, "data/calibration.json")
        self.assertEqual(options.duration, 300)
        self.assertEqual(options.exploration_policy, "coverage")
        self.assertEqual(options.territory_size, 1500)
        self.assertEqual(
            options.docking,
            {"init": False, "go-home-strategy": "hard-blocked-edge"},
        )
        self.assertEqual(options.go_home_strategy, "hard-blocked-edge")

    def test_mapping_config_option_can_precede_mode(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text('{"map_file": "data/room_map.json"}')
            options = map_room.parse_args(["--config", str(config), "start"])

        self.assertEqual(options.mode, "start")
        self.assertEqual(options.map_file, "data/room_map.json")

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
                        "docking": {
                            "go-home-strategy": "hard-blocked-edge",
                        },
                    }
                )
            )

            options = map_room.parse_args(["resume", "--config", str(config)])

        self.assertEqual(options.mode, "resume")
        self.assertEqual(options.map_file, "new.json")
        self.assertEqual(options.duration, 300)
        self.assertEqual(
            options.docking,
            {"init": True, "go-home-strategy": "hard-blocked-edge"},
        )
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
                "docking:\n"
                "  init: false\n"
                "  go-home-strategy: hard-blocked-edge\n"
            )

            options = map_room.parse_args(["dock", "--config", str(config)])

        self.assertEqual(options.map_file, "room_map.json")
        self.assertEqual(
            options.docking,
            {"init": False, "go-home-strategy": "hard-blocked-edge"},
        )
        self.assertEqual(options.go_home_strategy, "hard-blocked-edge")

    def test_mapping_config_rejects_root_go_home_strategy(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text(
                '{"map_file":"room_map.json","go_home_strategy":"hard-blocked-edge"}'
            )
            with self.assertRaises(SystemExit):
                map_room.parse_args(["dock", "--config", str(config)])

    def test_mapping_config_rejects_invalid_docking_init(self):
        with TemporaryDirectory() as directory:
            config = Path(directory) / "mapping.yaml"
            config.write_text(
                '{"map_file":"room_map.json","docking":{"init":"no"}}'
            )
            with self.assertRaises(SystemExit):
                map_room.parse_args(["start", "--config", str(config)])

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

    def test_start_with_existing_map_can_skip_initial_docking(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            map_file = root / "room_map.json"
            map_file.write_text(
                json.dumps(
                    {
                        "calibration": {
                            "deg_per_yaw": 0.5,
                            "mm_per_wheel_tick": 0.25,
                        },
                        "runs": [
                            {
                                "status": "accepted",
                                "path": [[11, 22, 33]],
                                "walls": [],
                                "obstacles": [],
                            }
                        ],
                    }
                )
            )
            config = root / "mapping.yaml"
            config.write_text(
                json.dumps(
                    {
                        "map_file": str(map_file),
                        "duration_seconds": 1,
                        "docking": {"init": False},
                    }
                )
            )
            run = {
                "status": "accepted",
                "path": [[11, 22, 33]],
                "walls": [],
                "obstacles": [],
            }

            with (
                patch.object(map_room, "send_command", return_value={"ok": True}),
                patch.object(map_room.time, "sleep"),
                patch.object(map_room, "dock_to_corner") as dock_to_corner,
                patch.object(map_room, "explore", return_value=run) as explore,
            ):
                map_room.main(["start", "--config", str(config)])

            dock_to_corner.assert_not_called()
            explore.assert_called_once()
            args = explore.call_args.args
            self.assertEqual(args[:5], (0.5, 0.25, 11.0, 22.0, 33.0))

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
                # Map frame mirrors the gyro handedness: the map-frame course
                # turns (-90 each) are commanded as +90, so the simulated gyro
                # reads positive yaw. The reconstructed map-frame path is
                # unchanged (the negation round-trips).
                side_effect=[
                    0, 250, 250,
                    90, 300, 200,
                    90, 550, 450,
                    180, 600, 400,
                    180, 850, 650,
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
                # Map frame mirrors the gyro handedness, so the simulated gyro
                # yaw is negated relative to the map-frame turns; the map-frame
                # return path is unchanged (the negation round-trips).
                side_effect=[
                    -90, -100, 100,
                    -90, 900, 1100,
                    0, 1100, 900,
                    90, 1300, 700,
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

    def test_forward_distance_is_integer_and_fits_remaining_time(self):
        self.assertEqual(map_room.forward_distance_for_remaining(60), 3000)
        self.assertEqual(map_room.forward_distance_for_remaining(2.5), 500)
        self.assertEqual(map_room.forward_distance_for_remaining(0.1), 200)
        self.assertEqual(map_room.home_leg_distance(452.429), 452)
        self.assertEqual(map_room.home_leg_distance(1200.5), 1000)

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

    def test_common_mode_slip_rejects_obstacle_stop_with_full_wheel_travel(self):
        issue = map_room.blocked_common_mode_slip_issue(
            "forward",
            325,
            337,
            motion_outcome={"halt": "obstacle", "phase": "moving"},
            front_blocked=True,
        )
        self.assertEqual(
            issue,
            "front obstacle stop with near-full wheel travel (suspected common-mode slip)",
        )

    def test_common_mode_slip_accepts_obstacle_stop_after_partial_progress(self):
        issue = map_room.blocked_common_mode_slip_issue(
            "forward",
            450,
            293,
            motion_outcome={"halt": "obstacle", "phase": "moving"},
            front_blocked=True,
        )
        self.assertIsNone(issue)

    def test_common_mode_slip_rejects_travel_faster_than_commanded(self):
        # 667 mm logged in only 0.5 s implies ~1334 mm/s, far over the 200 mm/s
        # command: the encoder over-read while blocked, so reject.
        issue = map_room.blocked_common_mode_slip_issue(
            "forward",
            725,
            667,
            motion_outcome={
                "halt": "obstacle",
                "phase": "moving",
                "elapsed_seconds": 0.5,
                "speed_mmps": 200,
            },
        )
        self.assertEqual(
            issue,
            "obstacle stop with wheel travel faster than commanded "
            "(suspected common-mode slip)",
        )

    def test_common_mode_slip_accepts_near_full_travel_consistent_with_time(self):
        # 667 mm over 3.34 s is exactly the commanded 200 mm/s -- the robot
        # genuinely drove almost to the target before meeting the obstacle, so
        # it is a normal wall stop and the pose is kept (no slip).
        issue = map_room.blocked_common_mode_slip_issue(
            "forward",
            725,
            667,
            motion_outcome={
                "halt": "obstacle",
                "phase": "moving",
                "elapsed_seconds": 3.34,
                "speed_mmps": 200,
            },
        )
        self.assertIsNone(issue)

    def test_common_mode_slip_legacy_fallback_rejects_short_blocked_probe(self):
        issue = map_room.blocked_common_mode_slip_issue(
            "forward",
            325,
            337,
            motion_outcome=None,
            front_blocked=True,
        )
        self.assertEqual(
            issue,
            "front obstacle with near-full wheel travel (suspected common-mode slip)",
        )

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
        # Correction is capped at LOOP_CLOSURE_MAX_CORRECTION_MM (a 100 mm
        # mismatch * 0.6 gain would be 60, but the bound is 50).
        self.assertEqual(dx, -50)
        self.assertEqual(dy, 0)

    def test_revisit_pose_correction_rejects_far_landmark_match(self):
        # A wall observation far from any known landmark is a different wall, not
        # drift, so it must not snap the pose toward it.
        self.assertGreater(200, map_room.LOOP_CLOSURE_MATCH_RADIUS_MM)
        self.assertIsNone(
            map_room.revisit_pose_correction(
                100,
                100,
                (300, 100),  # 200 mm from the only landmark
                [(100, 100)],
                [(500, 100)],
            )
        )

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

    def test_blocked_leg_cannot_commit_conservative_territory_transition(self):
        run = object.__new__(map_room._ExplorationRun)
        run.x, run.y, run.heading = 250, 100, 90
        run.path = [(250, -100, 90), (250, 100, 90)]
        run.known_path = [(250, -100), (250, 100)]
        run.events = [{}]
        run.policy = conservative.ConservativeExploration(
            [], (250, -100), [(250, -100)], [], territory_mm=1000
        )

        self.assertTrue(
            run.reject_blocked_territory_transition((250, -100), 1)
        )
        self.assertEqual((run.x, run.y), (250, -100))
        self.assertEqual(run.known_path, [(250, -100)])
        self.assertEqual(
            run.events[-1]['territory_transition_rejected'],
            {'from': (0, -1), 'to': (0, 0)},
        )

    def test_blocked_common_mode_slip_rolls_back_same_territory_leg(self):
        run = object.__new__(map_room._ExplorationRun)
        run.x, run.y, run.heading = 873, 853, 82
        run.path = [(827, 518, 84), (873, 853, 82)]
        run.known_path = [(827, 518), (873, 853)]
        run.baseline_pitch = 0
        run.events = [
            {
                "action": "forward",
                "requested": 325,
                "distance_mm": 337,
                "accepted": True,
            }
        ]
        run.quality = {
            "tracking_lost": False,
            "rejected_updates": 0,
            "issues": [],
        }

        with patch.object(
            map_room,
            "send_command",
            side_effect=[
                {"result": map_room.PROX_THRESHOLD},
                {"result": 0},
                {"result": 0},
            ],
        ):
            left, right, tilt, reason, sounds, back_away = run.handle_leg_end(
                337,
                325,
                previous_position=(827, 518),
                known_path_len=1,
                motion_outcome={"halt": "obstacle", "phase": "moving"},
            )

        self.assertEqual((left, right, tilt), (map_room.PROX_THRESHOLD, 0, 0))
        self.assertEqual(reason, "odometry rejected")
        self.assertIsNone(sounds)
        self.assertFalse(back_away)
        self.assertEqual((run.x, run.y), (827, 518))
        self.assertEqual(run.path[-1], (827, 518, 82))
        self.assertEqual(run.known_path, [(827, 518)])
        self.assertTrue(run.quality["tracking_lost"])
        self.assertEqual(run.quality["rejected_updates"], 1)
        self.assertFalse(run.events[-1]["accepted"])
        self.assertIn("common_mode_slip_rejected", run.events[-1])

    def test_strategy_avoids_known_blockers(self):
        blockers = [(400, 0), (800, 0), (1200, 0)]
        angle = map_room.choose_exploration_angle(0, 0, 0, blockers)
        self.assertNotEqual(angle, 0)

    def test_core_strategy_avoids_inferred_continuous_wall(self):
        walls = [(400, -100), (400, 100)]
        wall_segments = map_room.inferred_wall_segments(walls)
        angle = map_room.choose_exploration_angle(
            0,
            0,
            0,
            [],
            wall_segments=wall_segments,
        )
        self.assertNotEqual(angle, 0)

    def test_strategy_turns_away_from_live_proximity(self):
        left_blocked = map_room.choose_exploration_angle(
            0, 0, 0, [], blocked_left=20, require_turn=True
        )
        right_blocked = map_room.choose_exploration_angle(
            0, 0, 0, [], blocked_right=20, require_turn=True
        )
        self.assertLess(left_blocked, 0)
        self.assertGreater(right_blocked, 0)

    def test_novelty_policy_rewards_unexplored_direction(self):
        # Novelty is now a policy preference, not baked into heading_score.
        explored_east = [(distance, 0) for distance in (400, 800, 1200, 1600)]
        policy = NoveltyExplorationPolicy(
            explored_east, map_room.STRATEGY_SAMPLE_DISTANCES
        )
        angle = map_room.choose_exploration_angle(
            0, 0, 0, [], require_turn=False,
            heading_preference=policy.heading_preference,
        )
        self.assertNotEqual(angle, 0)

    def test_conservative_strategy_turns_before_mental_wall(self):
        angle = map_room.choose_exploration_angle(
            1800,
            1000,
            0,
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
                exploration_policy="novelty",
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
                # Map frame mirrors the gyro handedness, so a map-frame turn is
                # commanded as its negation and the simulated gyro reads the
                # opposite-signed yaw (here the turn leg's -100).
                side_effect=[
                    0, 100, 100,
                    0, 50, 50,
                    -100, 0, 100,
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

    def test_fresh_positive_start_explores_toward_positive_y(self):
        # Part B: from a freshly docked positive-quadrant start (territory
        # (0,0)), a map-frame left turn (+90) is commanded to hardware as -90;
        # the gyro reads -90 and the mirrored pose turns to map heading +90, so
        # the forward leg grows the path toward +y (into the room).
        readings = {
            "get_yaw": iter([0]),
            "get_left_wheel": iter([0]),
            "get_right_wheel": iter([0]),
            "get_pitch": iter([0] * 8),
            "get_prox_left": iter([0] * 4),
            "get_prox_right": iter([0] * 4),
        }

        def send_command(method, *args, **kwargs):
            if method in readings:
                return {"result": next(readings[method])}
            return {"result": None}

        times = iter([0.0, 0.0, 0.0, 0.5, 1.1, 1.1])
        angles = iter([90])
        with (
            patch.object(map_room, "send_command", side_effect=send_command),
            patch.object(map_room.time, "time", side_effect=lambda: next(times)),
            patch.object(
                map_room,
                "choose_exploration_angle",
                side_effect=lambda *a, **k: next(angles, 0),
            ),
            patch.object(map_room.random, "choice", return_value="okay"),
            patch.object(
                map_room,
                "read_settled",
                side_effect=[
                    -90, 50, -50,    # turn: map +90 commanded as -90, gyro -90
                    -90, 150, 50,    # forward +100 each wheel at heading +90 -> +y
                ],
            ),
        ):
            run = map_room.explore(1.0, 1.0, 310.0, 310.0, duration=1)

        self.assertEqual(run["status"], "accepted")
        # Started in territory (0,0) and grew toward +y, never behind y=0.
        self.assertEqual(
            conservative.territory_cell(310.0, 310.0, 1000), (0, 0)
        )
        ys = [point[1] for point in run["path"]]
        self.assertGreater(max(ys), 310.0)
        self.assertTrue(all(y >= 0 for y in ys))

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
