import unittest

from apps.dashboard import server as dashboard


class DashboardTests(unittest.TestCase):
    def test_apply_live_move_appends_pose_frame(self):
        payload = dashboard.empty_payload()

        frame = dashboard.apply_live_move(
            payload,
            {"pose": [100, 200, 90], "duration": 0.25, "timestamp": "now"},
        )

        self.assertEqual(frame["x"], 100)
        self.assertEqual(frame["y"], 200)
        self.assertEqual(frame["heading"], 90)
        self.assertEqual(payload["path"], [[100, 200]])
        self.assertEqual(payload["durations"], [])

    def test_apply_live_move_records_transition_duration(self):
        payload = dashboard.empty_payload()
        dashboard.apply_live_move(payload, {"pose": [0, 0, 0]})
        dashboard.apply_live_move(payload, {"pose": [100, 0, 0], "duration": 0.5})

        self.assertEqual(payload["durations"], [0.5])

    def test_live_dashboard_exports_static_html(self):
        live = dashboard.LiveDashboard(title="Live")
        live.post_move({"pose": [10, 20, 30]})

        html = live.static_html()

        self.assertIn("<title>Live</title>", html)
        self.assertIn('"frames"', html)

    def test_move_forwards_wall_marks_cell_blocked(self):
        payload = dashboard.empty_payload(territory_mm=1000)
        dashboard.apply_live_move(payload, {"pose": [125, 125, 0]})
        dashboard.apply_live_move(
            payload, {"pose": [375, 125, 0], "walls": [[625, 125]]}
        )

        # The forwarded wall is recorded and resolves a blocked cell.
        self.assertEqual(payload["walls"], [[625.0, 125.0, 1]])
        cells = payload["frames"][-1]["cells"]["0,0"]
        self.assertEqual(cells["2,0"], "blocked")
        self.assertEqual(cells["0,0"], "visited")

    def test_export_replays_seeded_history_progressively(self):
        live = dashboard.LiveDashboard(title="Live", territory_mm=1000)
        # Resume: prior coverage of row cy=0 plus a prior wall, then one move.
        live.seed(
            {
                "path": [[125, 125], [375, 125], [625, 125]],
                "walls": [[875, 125]],
                "obstacles": [],
            }
        )
        live.post_move({"pose": [625, 375, 90]})

        export = dashboard.export_payload(live.snapshot())

        # The seed is consumed into leading frames rather than shown up front.
        self.assertEqual(export["seed_path"], [])
        self.assertEqual(len(export["durations"]), len(export["frames"]) - 1)
        # The prior wall now reveals partway through the retrace, not at frame 0.
        self.assertEqual(len(export["walls"]), 1)
        self.assertGreater(export["walls"][0][2], 0)
        # Frame 0 shows only where the retrace begins; coverage grows over frames.
        first = export["frames"][0]["cells"]["0,0"]
        last = export["frames"][-1]["cells"]["0,0"]
        visited_first = sum(1 for v in first.values() if v == "visited")
        visited_last = sum(1 for v in last.values() if v == "visited")
        self.assertEqual(visited_first, 1)
        self.assertGreater(visited_last, visited_first)

    def test_arc_geometry_rides_on_the_frame_and_into_the_export(self):
        payload = dashboard.empty_payload(territory_mm=1000)
        dashboard.apply_live_move(payload, {"pose": [0, 0, 0]})
        # Predicted arc endpoint carries no arc; the amend reveals the measured
        # curve geometry.
        dashboard.apply_live_move(payload, {"pose": [100, 100, 90]})
        self.assertNotIn("arc", payload["frames"][-1])
        dashboard.amend_last_move(
            payload,
            {"pose": [100, 100, 90], "arc": {"radius_mm": 100, "angle_deg": 90}},
        )
        self.assertEqual(payload["frames"][-1]["arc"], {"radius_mm": 100, "angle_deg": 90})

        # The standalone-animation export preserves the arc geometry.
        export = dashboard.export_payload(payload)
        self.assertEqual(export["frames"][-1]["arc"], {"radius_mm": 100, "angle_deg": 90})

    def test_arc_from_poses_reconstructs_curve_and_ignores_straight_legs(self):
        # A quarter circle of radius 100 from origin facing +x ends at
        # (100, 100) facing +90; the chord + sweep recover radius and angle.
        arc = dashboard.arc_from_poses([0, 0, 0], [100, 100, 90])
        self.assertIsNotNone(arc)
        self.assertAlmostEqual(arc["radius_mm"], 100.0, places=1)
        self.assertAlmostEqual(arc["angle_deg"], 90.0, places=1)
        # A straight forward leg (no rotation) and an in-place turn (no travel)
        # are not arcs.
        self.assertIsNone(dashboard.arc_from_poses([0, 0, 0], [500, 0, 0]))
        self.assertIsNone(dashboard.arc_from_poses([0, 0, 0], [0, 0, 90]))

    def test_import_draws_arc_legs_and_grows_territories_along_the_path(self):
        # A free-roaming run with no bounded-territory metadata: it drives
        # straight north across two territory edges, then sweeps a quarter arc.
        data = {
            "schema_version": 2,
            "runs": [
                {
                    "status": "partial",
                    "path": [
                        [310, 310, 90],
                        [310, 1500, 90],  # enters (0,1)
                        [310, 2500, 90],  # enters (0,2)
                        [410, 2600, 180],  # a curved leg (travels and rotates)
                    ],
                    "walls": [],
                    "obstacles": [],
                    "events": [],
                    "wall_follower": {
                        "radius_mm": 250,
                        "arc_deg": 360,
                        "wall_on_left": True,
                    },
                }
            ],
        }

        payload = dashboard.build_payload(data)

        # Territories the path enters are all drawn, not just the start one.
        territories = {tuple(t) for t in payload["territories"]}
        self.assertEqual(territories, {(0, 0), (0, 1), (0, 2)})
        # The straight legs carry no arc; the curved final leg does.
        self.assertNotIn("arc", payload["frames"][1])
        self.assertNotIn("arc", payload["frames"][2])
        self.assertIn("arc", payload["frames"][3])

    def test_amend_last_move_corrects_pose_and_adds_walls(self):
        payload = dashboard.empty_payload(territory_mm=1000)
        dashboard.apply_live_move(payload, {"pose": [125, 125, 0]})
        # Predicted leg target far ahead.
        dashboard.apply_live_move(payload, {"pose": [875, 125, 0], "duration": 3.0})
        self.assertEqual(len(payload["frames"]), 2)

        # Robot actually stopped short at a wall; amend the last frame in place.
        frame = dashboard.amend_last_move(
            payload,
            {"pose": [375, 125, 0], "duration": 2.0, "walls": [[625, 125]]},
        )
        self.assertEqual(len(payload["frames"]), 2)  # amended, not appended
        self.assertEqual(frame["x"], 375.0)
        self.assertEqual(payload["path"][-1], [375.0, 125.0])
        self.assertEqual(payload["durations"][-1], 2.0)
        self.assertEqual(payload["walls"], [[625.0, 125.0, 1]])
        # The amended frame re-resolves: robot cell visited, wall cell blocked.
        cells = payload["frames"][-1]["cells"]["0,0"]
        self.assertEqual(cells["1,0"], "visited")
        self.assertEqual(cells["2,0"], "blocked")

    def test_load_map_renders_and_exports_verbatim(self):
        live = dashboard.LiveDashboard(territory_mm=1000)
        map_data = {
            "schema_version": 2,
            "runs": [
                {
                    "status": "accepted",
                    "path": [[125, 125, 0], [375, 125, 0]],
                    "walls": [[875, 125]],
                    "obstacles": [],
                    "events": [{"action": "forward"}],
                    "quality": {"tracking_lost": False},
                    "conservative_exploration": {
                        "territory_size_mm": 1000,
                        "territories": [[0, 0]],
                        "focus_territory": [0, 0],
                    },
                }
            ],
        }

        payload = live.load_map(map_data)
        self.assertTrue(payload["frames"])
        # Export hands back the authoritative data, not a pose reconstruction.
        self.assertEqual(live.map_for_export(), map_data)

    def test_load_map_rejects_map_without_runs(self):
        live = dashboard.LiveDashboard()
        with self.assertRaises(ValueError):
            live.load_map({"runs": []})

    def test_export_falls_back_to_live_payload_synthesis(self):
        live = dashboard.LiveDashboard(territory_mm=1000)
        live.post_move({"pose": [125, 125, 0]})
        live.post_move({"pose": [375, 125, 0], "walls": [[875, 125]]})

        # No authoritative map uploaded -> synthesize one from the live payload.
        synth = live.map_for_export()
        self.assertEqual(len(synth["runs"]), 1)
        self.assertEqual(synth["runs"][0]["walls"], [[875.0, 125.0]])
        # And it re-imports cleanly through the same path.
        self.assertTrue(dashboard.LiveDashboard().load_map(synth)["frames"])

    def test_apply_seed_primes_prior_coverage(self):
        payload = dashboard.empty_payload(territory_mm=1000)
        # Column 0 visited; a wall barrier in column 1 cuts off columns 2-3.
        dashboard.apply_seed(
            payload,
            {
                "path": [[125, 125], [125, 375], [125, 625], [125, 875]],
                "walls": [[260, 125], [260, 375], [260, 625], [260, 875]],
                "obstacles": [],
            },
        )

        # A seed creates no animation frames but primes territories and reveals
        # known blockers from frame 0.
        self.assertEqual(payload["frames"], [])
        self.assertEqual(payload["territories"], [[0, 0]])
        self.assertTrue(payload["walls"] and all(w[2] == 0 for w in payload["walls"]))

        # The first live pose then resolves prior visited / blocked / unreachable.
        dashboard.apply_live_move(payload, {"pose": [125, 125, 90]})
        cells = payload["frames"][-1]["cells"]["0,0"]
        self.assertEqual(cells["0,0"], "visited")
        self.assertEqual(cells["1,0"], "blocked")
        self.assertEqual(cells["3,0"], "unreachable")

