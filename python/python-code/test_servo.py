import pigpio
from servo_controller import Scanner


def main():

    pi = pigpio.pi()

    if not pi.connected:
        print("Could not connect to pigpio daemon")
        return

    YAW_PIN = 17    # for yaw servo
    PITCH_PIN = 18  # for pitch servo

    scanner = Scanner(pi, YAW_PIN, PITCH_PIN)

    try:
        print("Starting scan...")
        scanner.scan()    # going through 6 positions
        print("Scan finished")

    except KeyboardInterrupt:
        print("Stopped by user")

    finally:
        # Stop PWM signals
        pi.set_servo_pulsewidth(YAW_PIN, 0)
        pi.set_servo_pulsewidth(PITCH_PIN, 0)

        pi.stop()  # Servos offline


if __name__ == "__main__":
    main()
