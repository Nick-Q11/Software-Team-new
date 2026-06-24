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


BASE_DIR = Path(__file__).resolve().parents[1]
sys.path.append(str(BASE_DIR))

from lidar_interface.wrapper import LidarSensor

uav = UAV.connect(
        use_sim=False,
        serial_device="/dev/ttyS0"
    )

time.sleep(2)
    
print("UAV connected.")
