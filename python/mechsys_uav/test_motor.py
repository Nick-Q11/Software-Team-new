import asyncio
from mechsys_uav import UAV

async def main():

    uav = await UAV.connect(
        use_sim=False,
        serial_device="/dev/cu.usbmodem01"
    )

    print("Connected")

    await asyncio.sleep(2)

    print("Arming motors...")
    await uav._UAV__system.action.arm()

    print("Waiting 5 seconds...")
    await asyncio.sleep(5)

    print("Disarming...")
    await uav._UAV__system.action.disarm()

if __name__ == "__main__":
    asyncio.run(main())