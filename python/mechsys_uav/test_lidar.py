import asyncio
from mechsys_uav import UAV
import pigpio
from servo import Scanner
import sys
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parents[1]
sys.path.append(str(BASE_DIR))

from lidar_interface.wrapper import LidarSensor


async def vibration_test(lidar_sensor: LidarSensor):
    
    
    gimbal = Scanner()
    gimbal.center()
    
    zone = 64;
    closest_distance = 4000
    
    print("Vibrationstest gestartet. Überwache LiDAR-Matrix...")
    
    try:
        while True:
            closest_zone, distance = lidar_sensor.get_closest_zone()
            print(f"Zone {zone} hat den Unterschied {closest_distance-distance} mm")
            zone = closest_zone
            closest_distance = distance
            
            await asyncio.sleep(0.2)
            print()
    except KeyboardInterrupt:
        print("Vibrationstest beendet.")
    finally:
        gimbal.center()
        print("Gimbal in Mittelstellung zurückgesetzt.")
    
    

async def main():
    
    uav = await UAV.connect(
        use_sim=False,
        serial_device="/dev/cu.usbmodem01"
    )
    
    print("UAV connected.")
    await asyncio.sleep(2)
    
    uav.arm_and_takeoff(2)  # Take off to 2 meters altitude
    
    lidar_sensor = LidarSensor()
    if lidar_sensor.init_and_calibrate() != 0:
        print("Kritischer Fehler: Sensor-Kalibrierung fehlgeschlagen!")
        return
    print("Sensor erfolgreich kalibriert und bereit!")
    
    await vibration_test(lidar_sensor)
    
    uav.land()
    
if __name__ == "__main__":
    asyncio.run(main())
    
    
    
    
    