import asyncio
import numpy as np
from mechsys_uav import UAV
import haversine
import search
import search2
import telemetry
import pigpio

PIN_SERVO_PITCH = 12
PIN_SERVO_YAW = 13