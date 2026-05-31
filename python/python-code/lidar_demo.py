import asyncio
import sys
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parents[1]
sys.path.append(str(BASE_DIR))

from lidar_interface.wrapper import LidarSensor

async def autonomous(sensor: LidarSensor):
    print("Flugschleife gestartet. �berwache LiDAR-Matrix...")
    while True:
      
        closest_zone, distance = sensor.get_closest_zone()
        
       
        if 0 < distance < 400:
            print(f"?? NOT-STOPP! Hindernis in Zone {closest_zone} erkannt! Distanz: {distance} mm")
           
        else:
            
            print(f"Weg frei. N�chstes Objekt in Zone {closest_zone} ({distance} mm)")

        
        await asyncio.sleep(0.05)
        if distance < 10:
            break
        
async def material_handling(sensor: LidarSensor):
    print("Materialhandling started")
    for i in range(5):
        zone = sensor.get_zone_most_spads()
        spads = sensor.get_spads_of_zone(zone)
        print(f"Zone {zone} hat {spads} SPADs")
    

async def main():
    print("Initialisiere LiDAR Sensor...")
    sensor = LidarSensor()
  
    if sensor.init_and_calibrate() != 0:
        print("Kritischer Fehler: Sensor-Kalibrierung fehlgeschlagen!")
        return
        
    print("Sensor erfolgreich kalibriert und bereit!")
    
  
    await autonomous(sensor)
    

if __name__ == "__main__":
    asyncio.run(main())