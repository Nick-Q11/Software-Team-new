import asyncio
from mechsys_uav import UAV
import json
import asyncio
from importlib.resources import files
from shapely.geometry import Point, Polygon
from mavsdk import System
from mavsdk.telemetry import FlightMode
import pigpio
from servo import Scanner
import sys
from pathlib import Path
import time

async def main():

    uav = await UAV.connect(
            use_sim=False,
            serial_device="/dev/ttyS0"
            )
    
    print("UAV connected.")
    asyncio.sleep(2)

    uav.arm_and_takeoff(2)


if __name__ == "__main__":
    asyncio.run(main())
