import unittest

from examples.hardware_demo import parse_args as parse_hardware_demo_args
from examples.lightshow import parse_args as parse_lightshow_args
from examples.sensor_monitor import parse_args as parse_sensor_monitor_args


class ExampleArgumentTests(unittest.TestCase):
    def test_examples_default_to_doodle(self):
        for parse_args in (
            parse_hardware_demo_args,
            parse_lightshow_args,
            parse_sensor_monitor_args,
        ):
            with self.subTest(parse_args=parse_args.__module__):
                options = parse_args([])
                self.assertEqual(options.name, "Doodle")
                self.assertIsNone(options.address)

    def test_examples_accept_direct_address(self):
        for parse_args in (
            parse_hardware_demo_args,
            parse_lightshow_args,
            parse_sensor_monitor_args,
        ):
            with self.subTest(parse_args=parse_args.__module__):
                options = parse_args(["--address", "AA:BB:CC:DD:EE:FF"])
                self.assertEqual(options.address, "AA:BB:CC:DD:EE:FF")


if __name__ == "__main__":
    unittest.main()
