import json
import unittest
from datetime import datetime
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from examples.mapping import calibrate
from examples.mapping import map_room


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

    def test_output_argument_accepts_exact_file_path(self):
        self.assertEqual(
            calibrate.parse_args(["--output", "data/my_calibration.json"]).output,
            "data/my_calibration.json",
        )
        self.assertEqual(
            map_room.parse_args(["--output", "data/my_map.json"]).output,
            "data/my_map.json",
        )

    def test_duration_argument_accepts_positive_seconds(self):
        self.assertEqual(map_room.parse_args([]).duration, 60)
        self.assertEqual(map_room.parse_args(["--duration", "90"]).duration, 90)
        self.assertEqual(map_room.parse_args(["--duration", "2.5"]).duration, 2.5)
        with self.assertRaises(SystemExit):
            map_room.parse_args(["--duration", "0"])

    def test_latest_calibration_file_uses_timestamped_name(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            older = root / "calibration_20260611-23-59-59.json"
            newer = root / "calibration_20260612-14-05-09.json"
            legacy = root / "calibration.json"
            for path in (newer, legacy, older):
                path.write_text("{}")
            self.assertEqual(map_room.latest_calibration_file(root), newer)

    def test_go_home_argument_accepts_latest_or_explicit_map(self):
        self.assertEqual(map_room.parse_args(["--go-home"]).go_home, "latest")
        self.assertEqual(
            map_room.parse_args(["--go-home", "room_map.json"]).go_home,
            "room_map.json",
        )
        self.assertEqual(
            map_room.parse_args(["--start-with-map"]).start_with_map,
            "latest",
        )
        self.assertEqual(
            map_room.parse_args(
                ["--start-with-map", "room_map.json"]
            ).start_with_map,
            "room_map.json",
        )
        self.assertEqual(
            map_room.parse_args(
                ["--start-with-map", "room_map.json", "--calibration", "calibration.json"]
            ).calibration,
            "calibration.json",
        )

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
            map_room.plan_home_route(data)

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
        self.assertIn(("move", (1000, map_room.FORWARD_SPEED_MMPS), {"wall_stop_sound": None}), calls)

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
            map_room.dock_to_corner(1.0, 1.0)

        turns = [args[0] for method, args, _ in calls if method == "turn"]
        self.assertEqual(turns, [90, -90])

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
