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
import telemetry_client

async def main():
    client = telemetry_client.TelemetryClient(server_ip = "127.0.0.1", server_port = 5000)
    client.start()
    client.update_location(4.0, 10.0)
    
    await asyncio.sleep(5)  # Wait for a few seconds to allow the client to send data
    client.update_location(5.0, 11.0)
    await asyncio.sleep(5)  # Wait for a few seconds to allow the client to send
    
    client.stop()
    
if __name__ == "__main__":
    asyncio.run(main())
    
