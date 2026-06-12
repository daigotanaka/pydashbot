"""Run a light and head-motion show on a physical Dash robot."""

import argparse
import asyncio
import logging
import random

from dash.robot import DEFAULT_ROBOT_NAME, DashRobot, discover_and_connect


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    robot = parser.add_mutually_exclusive_group()
    robot.add_argument(
        "--name", default=DEFAULT_ROBOT_NAME, help="Bluetooth name to discover"
    )
    robot.add_argument("--address", help="Bluetooth address to connect to directly")
    return parser.parse_args(args)

async def run_light_show(robot):
    # Define a list of rave colors for the light show
    colors = ['#FF0000', '#00FF00', '#0000FF', '#FFFF00', '#FF00FF', '#00FFFF', '#FFFFFF', '#FFA500', '#FF1493', '#8A2BE2', '#00CED1', '#32CD32', '#FFD700', '#FF4500']
    
    # Run the rave light show
    logging.info("Starting rave light show...")
    for _ in range(50):  # Run the show for 50 cycles
        # Generate random colors for each light
        neck_color = random.choice(colors)
        ear_color = random.choice(colors)
        eye_pattern = random.randint(1, 4095)  # Random pattern for eye LEDs
        tail_brightness = random.randint(150, 255)  # Random brightness for tail light
        
        # Set random colors and patterns for each light
        if hasattr(robot, 'neck_color'):
            await robot.neck_color(neck_color)
        if hasattr(robot, 'left_ear_color') and hasattr(robot, 'right_ear_color'):
            await robot.left_ear_color(ear_color)
            await robot.right_ear_color(ear_color)
        if hasattr(robot, 'eye'):
            await robot.eye(eye_pattern)
        if hasattr(robot, 'tail_brightness'):
            await robot.tail_brightness(tail_brightness)
        
        # Randomly move the head back and forth (head nodding) synchronized with the lights
        await asyncio.sleep(0.5)  # Delay for synchronized movement
        await robot.head_yaw(30)  # Turn head to one side
        await asyncio.sleep(0.5)  # Delay for synchronized movement
        await robot.head_yaw(-30)  # Turn head to the other side
        await asyncio.sleep(0.5)  # Delay for synchronized movement
        await robot.head_yaw(0)  # Return head to center position
    
    # Turn off all lights at the end
    if hasattr(robot, 'neck_color'):
        await robot.neck_color('#000000')  # Turn off neck light
    if hasattr(robot, 'left_ear_color') and hasattr(robot, 'right_ear_color'):
        await robot.left_ear_color('#000000')  # Turn off left ear light
        await robot.right_ear_color('#000000')  # Turn off right ear light
    if hasattr(robot, 'eye'):
        await robot.eye(0)  # Turn off eye LEDs
    if hasattr(robot, 'tail_brightness'):
        await robot.tail_brightness(0)  # Turn off tail light
    
    logging.info("Rave light show completed.")

async def main(name=DEFAULT_ROBOT_NAME, address=None):
    logging.basicConfig(level=logging.INFO)
    robot = await discover_and_connect(name=name, address=address)
    if robot is None:
        logging.error("Failed to connect to a robot.")
        return
    if not isinstance(robot, DashRobot):
        logging.error("This light show requires a Dash robot.")
        await robot.disconnect()
        return

    try:
        await run_light_show(robot)
    finally:
        await robot.stop()
        await robot.disconnect()

if __name__ == "__main__":
    options = parse_args()
    asyncio.run(main(name=options.name, address=options.address))
