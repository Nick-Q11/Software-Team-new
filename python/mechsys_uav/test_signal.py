import pigpio
import asyncio


pi = pigpio()
async def main():
    try:
        while True:
            pin = 5
            pin.set_mode(pin, pigpio.OUTPUT)

            pi.write(pin, 1)
    except KeyboardInterrupt:
        print("End")

if __name__ == "__main__":
    asyncio.run(main())



