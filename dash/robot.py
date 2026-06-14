import asyncio
from bleak import BleakClient, BleakScanner
import binascii
import logging
from dash.constants import COMMANDS, COMMAND1_CHAR_UUID
from dash.sensors import RobotSensors
from collections import defaultdict
from dash.actuators import (
    CommonActuators,
    DashActuators,
    PoseDirection,
    PoseMode,
)
from dash.motion import MotionController

DEFAULT_ROBOT_NAME = "Doodle"


class Robot(CommonActuators):
    """BLE-backed common actuators and sensor access."""
    def __init__(self, address):
        self.address = address
        self.client = None
        self.sense = None
        self.sensor_state = defaultdict(int)

    async def connect(self):
        self.client = BleakClient(self.address)
        await self.client.connect()
        print(f'Connected to {self.address}')

        self.sense = RobotSensors(self.client, self.sensor_state)
        await self.sense.start()


    async def disconnect(self):
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            logging.info(f"Disconnected from {self.address}")

    async def reconnect(self):
        """Tear down a dropped link and connect again, restarting sensors."""
        try:
            await self.disconnect()
        except Exception as error:
            logging.warning(f"Error tearing down stale connection: {error}")
        await self.connect()


    async def command(self, command_name, command_values):  # Use `self` instead of `client`
            if self.client.is_connected:  # Access `is_connected` through `self.client`
                try:
                    char_uuid = COMMAND1_CHAR_UUID
                    message = bytearray([COMMANDS[command_name]]) + command_values
                    logging.debug(f"Sending command: {binascii.hexlify(message)}")
                    await self.client.write_gatt_char(char_uuid, message)  # Use `self.client` to write
                    logging.info("Command sent successfully.")
                except Exception as e:
                    logging.error(f"Failed to write to characteristic {char_uuid}: {str(e)}")

    # ------------ Sensor Getters for Dot ------------

    def get_time(self) -> float:
        '''
        Returns Dot's/Dash's current Unix time in seconds.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["dot_time"]

    def get_index(self) -> int:
        '''
        Unknown.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["dot_index"]

    def get_pitch(self) -> int:
        '''
        Returns Dot's/Dash's current pitch rotation.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["pitch"]

    def get_roll(self) -> int:
        '''
        Returns Dot's/Dash's current roll rotation.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["roll"]

    def get_acceleration(self) -> int:
        '''
        Returns Dot's/Dash's current acceleration.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["acceleration"]

    def is_button_white_pressed(self) -> bool:
        '''
        Returns if Dot's/Dash's white button is being pressed.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["button0"]

    def is_button_1_pressed(self) -> bool:
        '''
        Returns if Dot's/Dash's button one is being pressed.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["button1"]

    def is_button_2_pressed(self) -> bool:
        '''
        Returns if Dot's/Dash's button two is being pressed.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["button2"]

    def is_button_3_pressed(self) -> bool:
        '''
        Returns if Dot's/Dash's button three is being pressed.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["button3"]

    def is_moving(self) -> bool:
        '''
        Returns if Dot/Dash is moving.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["moving"]

    def is_picked_up(self) -> bool:
        '''
        Returns if Dot/Dash is being picked up.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["picked_up"]

    def is_hit(self) -> bool:
        '''
        Returns if Dot/Dash has been hit.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["hit"]

    def is_on_side(self) -> bool:
        '''
        Returns if Dot/Dash is laying or tilted on its side.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["side"]

    def is_nominal(self) -> bool:
        '''
        Unknown.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["nominal"]

    def has_heard_clap(self) -> bool:
        '''
        Returns if Dot/Dash heard a clap in the real world.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["clap"]

    def get_mic_level(self) -> int:
        '''
        Returns Dot's/Dash's microphone input level.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["mic_level"]

    def is_dot_left_of_dash(self) -> bool:
        '''
        Returns if Dot is to the left of Dash.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["dot_left_of_dash"]

    def is_dot_right_of_dash(self) -> bool:
        '''
        Returns if Dot is to the right of Dash.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["dot_right_of_dash"]

    def get_robot(self) -> str:
        '''
        Returns which robot type has been connected.
        '''
        if self.sense.dot_data_stream_ready:
            return self.sensor_state["robot"]


class DashRobot(MotionController, DashActuators, Robot):
    """Dash hardware composed with high-level bounded motion control."""

    def __init__(self, address):
        super().__init__(address)
        # Additional Dash-specific initialization

    # ------------ Sensor Getters for Dash ------------

    def get_dash_time(self) -> float:
        '''
        Returns Dash's current Unix time in seconds (same as get_time).
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["dash_time"]

    def get_dash_index(self) -> int:
        '''
        Unknown. Most times appear as get_index - 1
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["dash_index"]

    def get_pitch_delta(self) -> int:
        '''
        Returns the change in pitch, aka delta pitch or Δp, of Dash.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["pitch_delta"]

    def get_roll_delta(self) -> int:
        '''
        Returns the change in roll, aka delta roll or Δr, of Dash.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["roll_delta"]

    def get_prox_right(self) -> int:
        '''
        Returns the current distance of an object detected on the right proximity sensor of Dash.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["prox_right"]

    def get_prox_left(self) -> int:
        '''
        Returns the current distance of an object detected on the left proximity sensor of Dash.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["prox_left"]

    def get_prox_rear(self) -> int:
        '''
        Returns the current distance of an object detected on the rear proximity sensor of Dash.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["prox_rear"]

    def get_yaw(self) -> int:
        '''
        Returns Dash's current yaw rotation.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["yaw"]

    def get_yaw_delta(self) -> int:
        '''
        Returns the change in yaw, aka delta yaw or Δy, of Dash.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["yaw_delta"]

    def get_left_wheel(self) -> int:
        '''
        Returns the current rotation of Dash's left wheel, from 0 - 65535.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["left_wheel"]

    def get_right_wheel(self) -> int:
        '''
        Returns the current rotation of Dash's right wheel, from 0 - 65535.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["right_wheel"]

    def get_head_pitch(self) -> int:
        '''
        Returns Dash's head's current pitch rotation.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["head_pitch"]

    def get_head_yaw(self) -> int:
        '''
        Returns Dash's head's current yaw rotation.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["head_yaw"]

    def get_wheel_distance(self) -> int:
        '''
        Returns Dash's current distance traveled.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["wheel_distance"]

    def get_sound_direction(self) -> int:
        '''
        Returns the direction of a loud sound detected from Dash's microphone in yaw.
        '''
        if self.sense.dash_data_stream_ready:
            return self.sensor_state["sound_direction"]




async def discover_and_connect(
    retry_attempts=3,
    retry_delay=5,
    name=DEFAULT_ROBOT_NAME,
    address=None,
):
    """Connect to Dash by Bluetooth address or discover it by name."""
    attempt = 0
    while attempt < retry_attempts:
        attempt += 1
        action = "Connection" if address else "Discovery"
        logging.info(f"{action} attempt {attempt} of {retry_attempts}")
        try:
            if address:
                dash_robot = DashRobot(address)
                await dash_robot.connect()
                return dash_robot

            devices = await BleakScanner.discover()
            for device in devices:
                if device.name in {"Dash", "Dashet", name}:
                    logging.info(f"Found Dash at: {device.address}")
                    dash_robot = DashRobot(device.address)
                    await dash_robot.connect()
                    return dash_robot

            logging.warning("Dash not found. Retrying...")
        except Exception as e:
            logging.error(f"An error occurred while connecting to Dash: {e}")
        await asyncio.sleep(retry_delay)
    logging.error("Failed to connect to Dash after multiple attempts.")
    return None
