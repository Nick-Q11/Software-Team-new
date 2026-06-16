import asyncio
import json

from mechsys_uav import UAV

from shapely.geometry import Point, Polygon
from mavsdk import System
from mavsdk.telemetry import FlightMode
import haversine
import numpy as np

async def main():
    uav = await UAV.connect(serial_device='/dev/ttyACM0', serial_baud=57600, use_sim=False)
    print("UAV connected.")
    while True:
        lat, long, alt = uav.get_position()
        print(f"lat: {lat}, long: {long}, alt: {alt}")
        
        await asyncio.sleep(1)
        
if __name__ == '__main__':
    asyncio.run(main())