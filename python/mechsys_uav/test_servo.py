import asyncio
import pigpio
from servo import Scanner

async def main():
    pi = pigpio.pi()
    print("1")

    scanner = Scanner(pi)
    print("2")
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
        print("finally----------------------------------")
        scanner.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
