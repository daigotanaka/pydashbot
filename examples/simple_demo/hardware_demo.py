"""Exercise public actuator and high-level motion APIs on a physical robot."""

import argparse
import asyncio

from dash.core.robot import DEFAULT_ROBOT_NAME, DashRobot, discover_and_connect


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    robot = parser.add_mutually_exclusive_group()
    robot.add_argument(
        "--name", default=DEFAULT_ROBOT_NAME, help="Bluetooth name to discover"
    )
    robot.add_argument("--address", help="Bluetooth address to connect to directly")
    return parser.parse_args(args)

# Function to test Dash-specific movements and interactions
async def test_dash_movements(dash_robot):
    print("Testing Dash-specific movements...")

    print("Moving forward with obstacle-aware bounded motion...")
    await dash_robot.move(150, 100)

    print("Turning right...")
    await dash_robot.turn(-45)

    print("Turning left...")
    await dash_robot.turn(45)

    print("Moving backward with obstacle-aware bounded motion...")
    await dash_robot.move(-150, 100)

    # Additional Dash capabilities
    print("Adjusting head yaw and pitch...")
    await dash_robot.head_yaw(15)  # Slight turn right
    await asyncio.sleep(1)
    await dash_robot.head_yaw(-15)  # Slight turn left
    await asyncio.sleep(1)
    await dash_robot.head_pitch(5)  # Slight look up
    await asyncio.sleep(1)
    await dash_robot.head_pitch(-5)  # Slight look down
    await asyncio.sleep(1)

    print("Dash-specific movements test completed.")

# Function for basic interactions that both Dot and Dash can perform
async def test_basic_interactions(robot_instance):
    print("Testing basic interactions...")

    # LED and sound interactions
    print("Changing colors...")
    await robot_instance.neck_color("#FF00FF")  # Example color
    await asyncio.sleep(1)
    await robot_instance.left_ear_color("#00FF00")  # Example color
    await asyncio.sleep(1)
    await robot_instance.right_ear_color("#0000FF")  # Example color
    await asyncio.sleep(1)

    print("Playing a sound...")
    await robot_instance.say("hi")  # Play a sound
    await asyncio.sleep(1)

    print("Basic interactions test completed.")

# Main function to discover and test Dash or Dot
async def main(name=DEFAULT_ROBOT_NAME, address=None):
    robot = await discover_and_connect(name=name, address=address)
    if not robot:
        print("No compatible robot found.")
        return

    try:
        if isinstance(robot, DashRobot):
            print("Dash detected.")
            await test_dash_movements(robot)
        else:
            print("Dot detected.")

        await test_basic_interactions(robot)

    finally:
        # Ensuring graceful disconnect
        print("Cleaning up and disconnecting...")
        await robot.stop()
        await robot.reset(4)
        await robot.disconnect()
        print("Disconnected gracefully.")

if __name__ == "__main__":
    options = parse_args()
    asyncio.run(main(name=options.name, address=options.address))
