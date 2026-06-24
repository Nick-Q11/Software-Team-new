import asyncio
import sys
from pathlib import Path
import keyboard
import time


BASE_DIR = Path(__file__).resolve().parents[1]
sys.path.append(str(BASE_DIR))

from lidar_interface.wrapper import LidarSensor
trigger1 = 0

def trigger_f1():
    global trigger1
    
    if trigger1 == 0:
        trigger1 = 1

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
    global trigger1
    j = 0
    #keyboard.add_hotkey('space', trigger_f1)
    print("Press s for start.")
    #keyboard.wait('s')
    try:
       # while True:
        if trigger1 == 0:
            sensor.print_info_multiple(10)
            trigger1 = 0;
            time.sleep(1)

    except KeyboardInterrupt:
        print("Ende")
    

if __name__ == "__main__":
    asyncio.run(main())