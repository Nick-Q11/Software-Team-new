import asyncio
from mechsys_uav import UAV
import json
import asyncio
from importlib.resources import files
from shapely.geometry import Point, Polygon
from mavsdk import System
from mavsdk.telemetry import FlightMode



async def main():
    serial_device = "/dev/ttyS0"
    usb = 0
    if usb == 1:
        serial_device = serial_device.replace("S0", "ACM0")
    uav = await UAV.connect(
        use_sim=False,
        serial_device=serial_device
    )
    print("connected")
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

