import asyncio
import haversine as hav
from mechsys_uav import UAV
from servo import Scanner


# Flight Zone Corners


NORTH_EAST_CORNER = (49.57069804930975, 11.030361860205034)
NORTH_WEST_CORNER = (49.57058365455419, 11.030150838986174)
SOUTH_EAST_CORNER = (49.57057898537339, 11.030527456713145)
SOUTH_WEST_CORNER = (49.57046342304837, 11.030320035396016)


# Parameters


FLIGHT_ALTITUDE = 2.0
POSITION_TOLERANCE = 0.5

ROW_SPACING = 2.0     # meters


# Helper Functions


def get_abs_distance(position1, position2):
    return hav.haversine(
        position1[0:2],
        position2[0:2],
        unit=hav.Unit.METERS,
    )


def interpolate(start, end, t):
    return (
        start[0] + (end[0] - start[0]) * t,
        start[1] + (end[1] - start[1]) * t,
    )



# Flight Functions


async def takeoff(uav,
                  takeoff_altitude=FLIGHT_ALTITUDE,
                  vertical_uncertainity=0.2):

    accepted = await uav.arm_and_takeoff(
        takeoff_altitude=takeoff_altitude
    )

    while accepted:

        await asyncio.sleep(0.1)

        altitude = uav.get_position()[2]

        print(f"Altitude: {altitude:.2f} m")

        if altitude >= takeoff_altitude - vertical_uncertainity:
            print("Reached takeoff altitude.")
            break


async def fly_to_position(
        uav,
        goal_position,
        relative_altitude=FLIGHT_ALTITUDE,
        horizontal_uncertainity=POSITION_TOLERANCE):

    accepted = await uav.send_goal_position(
        goal_position[0],
        goal_position[1],
        relative_altitude,
    )

    while accepted:

        await asyncio.sleep(0.1)

        current_position = uav.get_position()

        distance = get_abs_distance(
            current_position,
            goal_position,
        )

        print(f"Distance: {distance:.2f} m")

        if distance <= horizontal_uncertainity:
            print("Reached waypoint.")
            break



# Search Path


def generate_search_path():

    height = get_abs_distance(
        NORTH_WEST_CORNER,
        SOUTH_WEST_CORNER,
    )

    rows = int(height / ROW_SPACING) + 1

    path = []

    for row in range(rows):

        t = row / (rows - 1)

        left = interpolate(
            NORTH_WEST_CORNER,
            SOUTH_WEST_CORNER,
            t,
        )

        right = interpolate(
            NORTH_EAST_CORNER,
            SOUTH_EAST_CORNER,
            t,
        )

        if row % 2 == 0:
            path.append(left)
            path.append(right)
        else:
            path.append(right)
            path.append(left)

    return path


# ----------------------------------------------------------------------
# Search Mission
# ----------------------------------------------------------------------

async def search(uav):

    path = generate_search_path()

    print("\nGenerated Search Path:\n")

    for i, point in enumerate(path):
        print(f"{i+1}: {point}")

    print()

    for i, waypoint in enumerate(path):

        print(f"Flying to waypoint {i+1}/{len(path)}")

        await fly_to_position(
            uav,
            waypoint,
        )


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

async def main():

    # Connect UAV
    uav = await UAV.connect(use_sim=True)

    await asyncio.sleep(2)

    print("Initial position:", uav.get_position())

    # Initialize Scanner
    pi = pigpio.pi()

    if not pi.connected:
        print("Could not connect to pigpio daemon.")
        return

    scanner = Scanner(pi)
    scanner.center()

    # Takeoff
    await takeoff(uav)

    # Start continuous scan
    scanner_task = asyncio.create_task(scanner.run())

    # Execute search mission
    await search(uav)

    print("\nMission complete.")

    # Stop scanner
    scanner_task.cancel()

    try:
        await scanner_task
    except asyncio.CancelledError:
        pass

    scanner.shutdown()

    # Land
    await asyncio.sleep(2)

    await uav.land()


if __name__ == "__main__":
    asyncio.run(main())