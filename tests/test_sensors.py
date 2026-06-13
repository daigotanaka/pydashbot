import unittest
from collections import defaultdict

from dash.sensors import RobotSensors


class SensorDecodeTests(unittest.TestCase):
    def setUp(self):
        self.state = defaultdict(int)
        self.sensors = RobotSensors(None, self.state)

    def test_dash_stream_decodes_proximity_and_wheel_distance(self):
        value = bytearray(20)
        value[6] = 11
        value[7] = 12
        value[8] = 13
        value[9] = 0x03
        value[10] = 0x56
        value[11] = 0x34

        self.sensors._dash_sensor_decode(None, value)

        self.assertEqual(self.state["prox_right"], 11)
        self.assertEqual(self.state["prox_left"], 12)
        self.assertEqual(self.state["prox_rear"], 13)
        self.assertEqual(self.state["wheel_distance"], 0x33456)

    def test_wheel_distance_remains_unsigned_above_sign_boundary(self):
        value = bytearray(20)
        value[9] = 0x0F
        value[10] = 0x56
        value[11] = 0x34

        self.sensors._dash_sensor_decode(None, value)

        self.assertEqual(self.state["wheel_distance"], 0xF3456)

    def test_dot_stream_decodes_motion_flags(self):
        value = bytearray(20)
        value[11] = 0x25

        self.sensors._dot_sensor_decode(None, value)

        self.assertTrue(self.state["picked_up"])
        self.assertTrue(self.state["hit"])
        self.assertTrue(self.state["side"])
        self.assertFalse(self.state["moving"])


if __name__ == "__main__":
    unittest.main()
