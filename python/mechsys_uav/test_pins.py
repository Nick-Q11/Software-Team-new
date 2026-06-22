import pigpio
import keyboard
import time

i = 0
pin = [5, 6, 19, 26, 12, 13]
pi = pigpio.pi()

def trigger():
    global i
    i = 1
    pi.write()

def main():
    j = 0
    keyboard.add_hotkey('space', trigger, )
    keyboard.wait('s')
    pin_name = ["PWR_EN", 29, "LPn", 31, "SPI_I2C_N", 35, "GPIO1", 37, "PWM1", 32, "PWM2", 33]
    for p in pin:
        pi.set_mode(p, pigpio.OUTPUT)
        pi.write(p, 0)
    try:
        while True:   
            if i == 1:
                pi.write(pin[j], 1)
                j = j+1
                print(f"pin name: {pin_name[2*j]} pin_phys: {pin_name[2*j+1]}")
                i = 0
            if keyboard.is_pressed('esc'):
                print("beendet: esc")
                break
    except KeyboardInterrupt:
        print("beendet: str+c")
        
    