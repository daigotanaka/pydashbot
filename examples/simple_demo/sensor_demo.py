"""Interactively inspect physical robot sensor readings."""

import argparse
import asyncio
from dash.robot import DEFAULT_ROBOT_NAME, DashRobot, discover_and_connect


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    robot = parser.add_mutually_exclusive_group()
    robot.add_argument(
        "--name", default=DEFAULT_ROBOT_NAME, help="Bluetooth name to discover"
    )
    robot.add_argument("--address", help="Bluetooth address to connect to directly")
    return parser.parse_args(args)


# Function for interactions with Dash's sensors
async def test_sensors(robot_instance: DashRobot):
    print("Testing sensor functionality...")
    print("Detecting movement on a flat surface. Starting in 3 seconds.\n")

    await asyncio.sleep(3)

    for _ in range(50): # Run checks for 50 cycles

        # Check if Dash or Dot is moving on the flat surface
        if robot_instance.is_moving():
            print(f"Cycle {_+1}/50 - Dash/Dot is moving.")
        else:
            print(f"Cycle {_+1}/50 - Dash/Dot is not moving.")
        
        await asyncio.sleep(0.3)

    print("\nDetecting Dash/Dot being picked up. Starting in 3 seconds.\n")
    
    await asyncio.sleep(3)

    for _ in range(50): # Run checks for 50 cycles

        # Check if Dash or Dot is being picked up
        if robot_instance.is_picked_up():
            print(f"Cycle {_+1}/50 - Dash is being picked up.")
        else:
            print(f"Cycle {_+1}/50 - Dash is set down.")
        

        await asyncio.sleep(0.3)

    print("\nDetecting rotation. Starting in 3 seconds.\n")
    
    await asyncio.sleep(3)

    for _ in range(10): # Run checks for 10 cycles

        # Print rotation values
        print(f"[Cycle {_+1}/10]")
        print(f"Pitch - {robot_instance.get_pitch()}")
        print(f"Roll - {robot_instance.get_roll()}")
        if robot_instance.get_robot() == "dash":
            print(f"Yaw - {robot_instance.get_yaw()}")
        print()

        await asyncio.sleep(1)

    print("\nDetecting button states. Starting in 3 seconds.\n")
    
    await asyncio.sleep(3)

    for _ in range(200): # Run checks for 200 cycles

        print(f"Cycle {_+1}/200 - ", end="")

        # Check for button states
        if robot_instance.is_button_white_pressed():
            print("[White]", end="")
        if robot_instance.is_button_1_pressed():
            print("[·]", end="")
        if robot_instance.is_button_2_pressed():
            print("[··]", end="")
        if robot_instance.is_button_3_pressed():
            print("[∴]", end="")

        print()

        await asyncio.sleep(0.1)


    print("\nDetecting microphone by clapping. Clap 4 times to test.\n")
        
    claps = 0

    # Get the current sensor data
    while claps < 4:
        # Check if a clap is heard
        if robot_instance.has_heard_clap():
            print("Detected a clap.")

            claps += 1

        await asyncio.sleep(0.1)

    print("\nSensor functionality test completed.\n")

# Main function to discover and test Dash or Dot
async def main(name=DEFAULT_ROBOT_NAME, address=None):
    robot = await discover_and_connect(name=name, address=address)
    if not robot:
        print("No compatible robot found.")
        return
    if not isinstance(robot, DashRobot):
        print("This test is only for Dash robots.")
        await robot.disconnect()
        return

    try:
        await test_sensors(robot)

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
