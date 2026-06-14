import math
import unittest

from dash.actuators import (
    MAX_TURN_CENTIRADIANS,
    TURN_DEADBAND_DEG_CCW,
    TURN_DEADBAND_DEG_CW,
    compensate_turn,
    encode_move,
)


def decode_distance(packet):
    value = packet[0] | ((packet[5] & 0x3F) << 8)
    return value - 0x4000 if value & 0x2000 else value


def decode_turn_centiradians(packet):
    value = packet[2] | (((packet[5] >> 6) & 0x03) << 8)
    return value - 0x400 if packet[6] & 0xC0 == 0xC0 else value


class MoveEncodingTests(unittest.TestCase):
    def test_turn_angle_is_deadband_compensated(self):
        packet = encode_move(0, 360, 1)

        self.assertEqual(decode_distance(packet), 0)
        self.assertEqual(
            decode_turn_centiradians(packet),
            int(math.radians(compensate_turn(360)) * 100),
        )

    def test_negative_turn_compensates_and_preserves_direction(self):
        packet = encode_move(0, -360, 1)

        self.assertEqual(decode_distance(packet), 0)
        self.assertEqual(
            decode_turn_centiradians(packet),
            int(math.radians(compensate_turn(-360)) * 100),
        )

    def test_large_distance_does_not_encode_a_turn(self):
        packet = encode_move(2048, 0, 1)

        self.assertEqual(decode_distance(packet), 2048)
        self.assertEqual(decode_turn_centiradians(packet), 0)

    def test_compensate_turn_extends_magnitude_in_command_direction(self):
        self.assertEqual(compensate_turn(0), 0)
        self.assertEqual(compensate_turn(90), 90 + TURN_DEADBAND_DEG_CCW)
        self.assertEqual(compensate_turn(-90), -(90 + TURN_DEADBAND_DEG_CW))

    def test_compensated_turn_is_clamped_below_field_rollover(self):
        # A near-max command plus the deadband would overflow the 10-bit field
        # and wrap to a tiny angle; it must clamp instead.
        packet = encode_move(0, 585, 1)

        self.assertEqual(decode_turn_centiradians(packet), MAX_TURN_CENTIRADIANS)


if __name__ == "__main__":
    unittest.main()
