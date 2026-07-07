import asyncio
from mechsys_uav import UAV
import pigpio
from servo import Scanner, Servo
import sys
from pathlib import Path
import json
from importlib.resources import files
from shapely.geometry import Point, Polygon
from mavsdk import System
from mavsdk.telemetry import FlightMode

BASE_DIR = Path(__file__).resolve().parents[1]
sys.path.append(str(BASE_DIR))

from lidar_interface.wrapper import LidarSensor




async def vibration_test(lidar_sensor: LidarSensor):   
    zone_d = 64
    closest_distance = 5000 # 3m als default-Wert
    zone_s = 64
    highest_spads = 0 # 0 als default Wert
    
    print("Vibrationstest gestartet. Überwache LiDAR-Matrix...")
    
    try:
        while True:
            print("Aktuelle LiDAR-Matrix:")
            lidar_sensor.print_info_matrix()
            closest_zone, distance = lidar_sensor.get_closest_zone()
            highest_spads_zone, spad = lidar_sensor.get_most_spads()
            print("-------------------Distanzamplitude:-------------------------")
            print(f"Zone_d {zone_d} hat den Unterschied {closest_distance-distance} mm")
            zone_d = closest_zone
            closest_distance = distance
            print("----------------Helligkeitsunterschied:----------------------")
            print(f"Zone_s {zone_s} hat den Unterschied {highest_spads-spad} SPADs")
            zone_s = highest_spads_zone
            highest_spads = spad
            await asyncio.sleep(1)
            print()
    except KeyboardInterrupt:
        print("Vibrationstest beendet.")
    finally:
        #gimbal.center()
        print("Gimbal in Mittelstellung zurückgesetzt.")
        
    


        
async def find_target(lidar_sensor: LidarSensor):
    try:
        while True:
            zone, spad = lidar_sensor.get_most_spads()
            print(f"Zone:  {zone} Spad:  {spad}")
            await asyncio.sleep(1)
            print()
    except KeyboardInterrupt:
        print("find_target beendet")

async def main():
    """
    uav = await UAV.connect(
        use_sim=False,
        serial_device="/dev/ttyS0"
    )
    
    print("UAV connected.")
    await asyncio.sleep(2)
    
    await uav.arm_and_takeoff(2)# Take off to 2 meters altitude
    position = await uav.get_position()
    await uav.check_goal_position(latitude=49.57080000000000, longitude=11.03025000000000, relative_altitude=2)
    await uav.send_goal_position(latitude=49.57080000000000, longitude=11.03025000000000, relative_altitude=2)
    await asyncio.sleep(5)
    await uav.send_goal_position(latitude=position[0], longitude=position[1], relative_altitude=position[2])
"""
    pi = pigpio.pi()
    pi.set_mode(19, pigpio.OUTPUT)
    await asyncio.sleep(1)
    pi.write(19, 1)
    lidar_sensor = LidarSensor()
    if lidar_sensor.init_and_calibrate() != 0:
        print("Kritischer Fehler: Sensor-Kalibrierung fehlgeschlagen!")
        return
    print("Sensor erfolgreich kalibriert und bereit!")
    #pitch.set_angle(45)
    await vibration_test(lidar_sensor)
    
    #pitch.set_angle(45)
    
    #await vibration_test(lidar_sensor)
    
    #uav.land()
    
if __name__ == "__main__":
    asyncio.run(main())
    
    
    
    
    