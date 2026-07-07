import asyncio
import haversine as hav
from mechsys_uav import UAV
from servo import Scanner
import pigpio
from marker_detector import MarkerDetector
import lidar_geometry
from telemetry_client import TelemetryClient
import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[1]
sys.path.append(str(BASE_DIR))

from lidar_interface.wrapper import LidarSensor

# Flight Zone Corners


NORTH_EAST_CORNER = (49.57069804930975, 11.030361860205034)
NORTH_WEST_CORNER = (49.57058365455419, 11.030150838986174)
SOUTH_EAST_CORNER = (49.57057898537339, 11.030527456713145)
SOUTH_WEST_CORNER = (49.57046342304837, 11.030320035396016)


# Parameters


FLIGHT_ALTITUDE = 2.0
POSITION_TOLERANCE = 0.5

ROW_SPACING = 2.0     # meters

ZONE_FOUND = [27, 28, 35, 36]  # Zones where the marker is expected to be found


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


async def detect_marker(marker_detector):
    marker = marker_detector.detect()

    if asyncio.iscoroutine(marker):
        marker = await marker

    return marker



# Flight Functions


async def takeoff(uav,
                  takeoff_altitude=FLIGHT_ALTITUDE,
                  vertical_uncertainity=0.2):

    accepted = await uav.arm_and_takeoff(
        takeoff_altitude=takeoff_altitude
    )

    if not accepted:
        print("Takeoff wurde vom UAV abgelehnt!")
        
        return False
    
    while accepted:

        await asyncio.sleep(0.1)
    
        position = uav.get_position()
    
        if position is None:
            print("Verbindung zum UAV während des Takeoffs verloren!")
            
            return False
    
        print(f"Altitude: {position[2]:.2f} m")
    
        if position[2] >= takeoff_altitude - vertical_uncertainity:
            print("Reached takeoff altitude.")
            
            return True




async def fly_to_position(
        uav,
        goal_position,
        relative_altitude=FLIGHT_ALTITUDE,
        horizontal_uncertainity=POSITION_TOLERANCE,
        marker_detector=None,
        marker_callback=None):

    accepted = await uav.send_goal_position(
        goal_position[0],
        goal_position[1],
        relative_altitude,
    )
    
    if not accepted:
        print(f"Flugbefehl zu {goal_position} wurde abgelehnt!")
        return False

    while accepted:

        await asyncio.sleep(0.1)

        current_position = uav.get_position()
        
        if current_position is None:
            print("Verbindung zum UAV während des Fluges verloren!")
            return False

        if marker_detector is not None and marker_callback is not None:
            marker = await detect_marker(marker_detector)
            if marker:
                stop_point = tuple(current_position)
                attitude = uav.get_attitude()
                handled = marker_callback(marker, stop_point, attitude)

                if asyncio.iscoroutine(handled):
                    handled = await handled

                if handled:
                    accepted = await uav.send_goal_position(
                        goal_position[0],
                        goal_position[1],
                        relative_altitude,
                    )

                    if not accepted:
                        print(f"Flugbefehl zu {goal_position} wurde abgelehnt!")
                        return False

                    continue

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


# Search Mission

async def fly_to_marker(uav: UAV, lidar: LidarSensor, scanner: Scanner, stop_point):
    position = stop_point
    found = False
    last_spad = None
    
    while True:
        marker = MarkerDetector(lidar).detect()
        if marker is None:
            found = False
            position = stop_point
            break
        last_spad = marker.spad
        
        if marker.zone in ZONE_FOUND:
            found = True
            break
        attitude = uav.get_attitude()
        position = lidar_geometry.lidar_to_gps(stop_point[0],
                                              stop_point[1],
                                              attitude[1],
                                              attitude[0],
                                              attitude[2],
                                              scanner.yaw.val,
                                              scanner.pitch.val,
                                              marker.zone,
                                              marker.distance)
        await fly_to_position(uav, position)
    await fly_to_position(uav, stop_point)
    
    if found:
        return position, last_spad
    
    return None, None

def check_marker(marker, found_marker_list, stop_point, attitude, scanner):
    if not found_marker_list:
        return False
    
    position = lidar_geometry.lidar_to_gps(stop_point[0],
                                            stop_point[1],
                                            attitude[1],
                                            attitude[0],
                                            attitude[2],
                                            scanner.yaw.val,
                                            scanner.pitch.val,
                                            marker.zone,
                                            marker.distance)
    
    position_tuple = (position[0], position[1])
    
    for known_gps in found_marker_list:
        known_marker = (known_gps[0], known_gps[1])
        radius = hav.haversine(position_tuple, known_marker, unit=hav.Unit.METERS)
        if radius < 2.0:
            return True
        
    return False
    

async def search(uav, lidar, scanner, client):
    known = False
    marker_position = None
    marker_detector = MarkerDetector(lidar)
    path = generate_search_path()
    found_marker_list = []
    found_marker_spad_list = []
    sort_marker_list = []

    async def handle_marker(marker, stop_point, attitude):
        nonlocal known, marker_position, sort_marker_list

        print(f"Marker detected in zone {marker.zone} with SPAD {marker.spad}")
        
        known = check_marker(marker, found_marker_list, stop_point, attitude, scanner)
           
        if known:
            print(f"Marker bekannt.")
            return False
            
        else:
            await fly_to_position(uav, stop_point,)
            
            marker_position, marker_spad = await fly_to_marker(uav, lidar, scanner, stop_point)
            
            if marker_position is not None:
                found_marker_list.append(marker_position)
                found_marker_spad_list.append(marker_spad)
                sort_marker_list = [x for _, x in sorted(zip(found_marker_spad_list, found_marker_list),
                                                         key=lambda x: x[0],
                                                         reverse=True)]
                best_marker = sort_marker_list[0]
                lat = best_marker[0]
                lon = best_marker[1]
                client.update_location(lat, lon)

        return True

    print("\nGenerated Search Path:\n")

    for i, point in enumerate(path):
        print(f"{i+1}: {point}")

    print()

    for i, waypoint in enumerate(path):
            

        print(f"Flying to waypoint {i+1}/{len(path)}")

        await fly_to_position(
            uav,
            waypoint,
            marker_detector=marker_detector,
            marker_callback=handle_marker,
        )
        
    
    return sort_marker_list
            
            
            
                

# Main


async def main():

    client = TelemetryClient(server_ip = "127.0.0.1", server_port = 5000)
    client.start()
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
    
    lidar = LidarSensor()

    # Takeoff
    await takeoff(uav)

    # Start continuous scan
    scanner_task = asyncio.create_task(scanner.continuous_scan())

    # Execute search mission
    marker_list = await search(uav, lidar, scanner, client)

    print("\nMission complete.")
    print("Found markers at the following positions sort by SPAD:")
    for i, position in enumerate(marker_list):
        print(f"{i+1}: {position}")

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
    
    client.stop()


if __name__ == "__main__":
    asyncio.run(main())
