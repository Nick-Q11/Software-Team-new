import asyncio
from mechsys_uav import UAV


async def main():

    uav = await UAV.connect(
        use_sim=False,
        serial_device="/dev/cu.usbmodem01"
    )

    await asyncio.sleep(2)

    while True:
        latitude,longitude,relative_altitude = uav.get_position()

        print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {relative_altitude}")

        await asyncio.sleep(1)

if __name__ == '__main__':
    asyncio.run(main())

