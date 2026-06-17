import asyncio
from servo import Scanner


async def main():

    scanner = Scanner()

    try:
        print("Centering...")
        scanner.center()

        await asyncio.sleep(1)

        print("Starting scan...")
        measurements = await scanner.scan()

        print("Scan finished")
        print(measurements)

    except KeyboardInterrupt:
        print("Stopped by user")

    finally:
        scanner.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
