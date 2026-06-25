import pigpio
import keyboard
import time
import sys

trigger1 = 0
pin = [14, 15, 10, 9, 11, 8, 5, 6, 12, 13, 19, 26, 20, 21]
pi = pigpio.pi()

if not pi.connected:
    sys.exit()

def trigger_f1():
    global trigger1
    
    if trigger1 == 0:
        trigger1 = 1

def main():
    global trigger1
    #keyboard.add_hotkey('space', trigger_f1)
    print("Press s for start.")
    #keyboard.wait('s')
    pin_name = ["UART-TX", "UART-RX", "MOSI", "MISO", "CLK", "CSe", "PWR_EN", "LPn", "PWM1", "PWM2", "SPI_I2C", "GPIO1", "res1", "res2"]
    pysical_pin = [8, 10, 19, 21, 23, 24, 29, 31, 32, 33, 35, 37, 38, 40]
    for gpio in range(2, 28):
        #print("GPIOs werden auf low gesetzt.")
        pi.set_mode(gpio, pigpio.OUTPUT)
        pi.write(gpio, 0)
    #print("Press space to get to next GPIO.\nPress esc to end program. \nPress strg+c to kill program")
    try:
        """
        while True:   
            if trigger1 == 0:
                if j == 0:
                    #print(f"GPIO: {pin[-1]} wird auf low gesetzt.")
                    pi.write(pin[-1], 0)
                else:
                    #print(f"GPIO: {pin[j-1]} wird auf low gesetzt.")
                    pi.write(pin[j-1], 0)
                
                #print(f"GPIO: {pin[j]} wird auf high gesetzt.")
                pi.write(pin[j], 1)
                print(f"pin name: {pin_name[j]} pin_phys: {pysical_pin[j]}")
                
                j = j+1
                trigger1 = 0
                if j >= (len(pin_name)):
                    print("Press space to get to first GPIO.")
                    j = 0
                    
            if keyboard.is_pressed('esc'):
                print("beendet: esc")
                break
            
            time.sleep(0.01)
            """
        for gpio in range(2, 28):
            pi.write(gpio, 0)
        j = 0
        pi.write(pin[j], 1)
        print(f"pin name: {pin_name[j]} pin_phys: {pysical_pin[j]}")
            
    except KeyboardInterrupt:
        print("beendet: str+c")

""" finally:
        #print("GPIOs werden auf low gesetzt.")
        for gpio in range(2, 28):
            pi.write(gpio, 0)
        pi.stop()
        """

if __name__ == "__main__":
    main()
    