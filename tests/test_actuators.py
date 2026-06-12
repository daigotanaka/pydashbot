import math
import unittest

from dash.actuators import encode_move


def decode_distance(packet):
    value = packet[0] | ((packet[5] & 0x3F) << 8)
    return value - 0x4000 if value & 0x2000 else value


def decode_turn_centiradians(packet):
    value = packet[2] | (((packet[5] >> 6) & 0x03) << 8)
    return value - 0x400 if packet[6] & 0xC0 == 0xC0 else value


class MoveEncodingTests(unittest.TestCase):
    def test_large_turn_does_not_encode_forward_distance(self):
        packet = encode_move(0, 360, 1)

        self.assertEqual(decode_distance(packet), 0)
        self.assertEqual(
            decode_turn_centiradians(packet),
            int(math.radians(360) * 100),
        )

    def test_negative_large_turn_preserves_direction(self):
        packet = encode_move(0, -360, 1)

        self.assertEqual(decode_distance(packet), 0)
        self.assertEqual(
            decode_turn_centiradians(packet),
            int(math.radians(-360) * 100),
        )

    def test_large_distance_does_not_encode_a_turn(self):
        packet = encode_move(2048, 0, 1)

        self.assertEqual(decode_distance(packet), 2048)
        self.assertEqual(decode_turn_centiradians(packet), 0)


if __name__ == "__main__":
    unittest.main()
