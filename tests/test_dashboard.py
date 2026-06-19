import unittest

from apps.map import dashboard


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

