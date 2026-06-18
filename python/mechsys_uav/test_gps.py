import asyncio
from mechsys_uav import UAV
import json
import asyncio
from importlib.resources import files
from shapely.geometry import Point, Polygon
from mavsdk import System
from mavsdk.telemetry import FlightMode



async def main():

    uav = await UAV.connect(
        use_sim=False,
        serial_device="/dev/ttyACM0"
    )

    await asyncio.sleep(2)
    try:
        while True:
            latitude,longitude,relative_altitude = uav.get_position()

            print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {relative_altitude}")

            await asyncio.sleep(5)
    except KeyboardInterrupt:
        print("GPS beendet")
if __name__ == '__main__':
    asyncio.run(main())

