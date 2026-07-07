import asyncio
import pigpio

from mechsys_uav import UAV
from servo import Scanner
from search import search


async def main():

    pi = pigpio.pi()

    scanner = Scanner(pi)

    uav = await UAV.connect(use_sim=False)

    home_position = uav.get_position()[:2]

    try:
        await search(
            uav=uav,
            home_position=home_position,
            scanner=scanner,
        )
    finally:
        scanner.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
