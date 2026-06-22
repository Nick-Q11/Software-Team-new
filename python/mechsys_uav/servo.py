import asyncio
import pigpio

YAW_PIN = 12
PITCH_PIN = 13


def angle_to_pwm_us(angle_deg: float) -> int:
    """
    Convert angle [0°,180°] to PWM pulse width [500µs,2500µs]
    """
    return int(500 + (angle_deg / 180.0) * 2000)


class Servo:
    def __init__(self, pi, pin):
        self.pi = pi
        self.pin = pin

    def set_angle(self, angle_deg: float):
        pulse = angle_to_pwm_us(angle_deg)
        self.pi.set_servo_pulsewidth(self.pin, pulse)

    def stop(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)


class Scanner:
    def __init__(self, pi):
        self.pi = pi

        self.yaw = Servo(pi, YAW_PIN)
        self.pitch = Servo(pi, PITCH_PIN)

    async def _serpentine_sweep(
        self,
        yaw_values,
        pitch_values,
        dwell_s=0.5,
    ):
        """
        Scan in serpentine pattern.

        Returns:
            List[(yaw, pitch)]
        """

        measurements = []

        pitch_rows = sorted(pitch_values, reverse=True)

        for row_idx, pitch in enumerate(pitch_rows):

            self.pitch.set_angle(90 + pitch)

            await asyncio.sleep(dwell_s)

            row_yaws = (
                yaw_values
                if row_idx % 2 == 0
                else yaw_values[::-1]
            )

            for yaw in row_yaws:

                self.yaw.set_angle(90 + yaw)

                await asyncio.sleep(dwell_s)

                measurements.append(
                    {
                        "yaw": yaw,
                        "pitch": pitch,
                    }
                )

        return measurements

    async def coarse_scan(self):
        """
        3 x 3 scan grid.
        """

        return await self._serpentine_sweep(
            yaw_values=[-90, 0, 90],
            pitch_values=[-90, -45, 0],
            dwell_s=0.5,
        )

    async def scan(self):
        """
        Main scan function.

        Returns:
            measurements:
            [
                {"yaw": ..., "pitch": ...},
                ...
            ]
        """

        measurements = await self.coarse_scan()

        return measurements

    def center(self):
        self.yaw.set_angle(90)
        self.pitch.set_angle(90)

    def shutdown(self):
        self.center()

        self.yaw.stop()
        self.pitch.stop()
        
async def main():
    pi = pigpio.pi()
    #yaw = Servo(pi, YAW_PIN)
    pitch = Servo(pi, PITCH_PIN)
    #yaw.set_angle(0)
    pitch.set_angle(0)
    
    
    #for i in range(20):
        
        #yaw.set_angle(i*9)
        #await asyncio.sleep(1)
    for i in range(10):
        pitch.set_angle(i*9)
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())