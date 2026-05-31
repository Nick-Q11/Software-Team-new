import time


def angle_to_pwm_us(angle_deg: float) -> int:
    """
    Convert an angle (in degrees) to a PWM pulse width (in microseconds)

    Assumptions:
    Servo angle range: 0° to 180°
    PWM range: 500 µs to 2500 µs

    Linear mapping:
    0°   -> 500 µs
    180° -> 2500 µs
    """
    return int(500 + (angle_deg / 180.0) * 2000)


class Servo:
    def __init__(self, pi, pin):
        self.pi = pi
        self.pin = pin

    def set_angle(self, angle_deg: float):
        pulse = angle_to_pwm_us(angle_deg)
        self.pi.set_servo_pulsewidth(self.pin, pulse)


class Scanner:
    def __init__(self, pi, yaw_pin, pitch_pin):
        self.yaw = Servo(pi, yaw_pin)
        self.pitch = Servo(pi, pitch_pin)

    def _serpentine_sweep(self, yaw_values, pitch_values, dwell_s=0.5):
        # Scan from upper rows to lower rows
        pitch_rows = sorted(pitch_values, reverse=True)
        for row_idx, pitch in enumerate(pitch_rows):
            # Move pitch servo
            self.pitch.set_angle(90 + pitch)
            # Move yaw servo in serpentine pattern
            row_yaws = yaw_values if row_idx % 2 == 0 else yaw_values[::-1]
            for yaw in row_yaws:
                self.yaw.set_angle(90 + yaw)
                # Wait for servo to settle
                time.sleep(dwell_s)

    def coarse_scan(self):
        self._serpentine_sweep([-90, 0, 90], [-90, -45, 0])

    def fine_scan(self):
        pass

    def scan(self):
        self.coarse_scan()
