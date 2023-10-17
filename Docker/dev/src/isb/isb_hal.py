# -*- coding: utf-8 -*-


from PCA95XX.PCA95XX import PCA95XX

from time import sleep, time_ns


# DEVICE_BUS = 1
# DEVICE_ADDR = 0x20
# bus = smbus.SMBus(DEVICE_BUS)
# bus.write_byte_data(DEVICE_ADDR, 0x00, 0x01)

# '''
# - First byte : command byte
# Defines the register we choose to change (from 0 to 7)

# 0 Input port 0 (read only, reflects the state of the GPIO (input or output))
# 1 Input port 1
# 2 Output port 0 (write only, )
# 3 Output port 1
# 4 Polarity Inversion port 0 (revert the polarity of the input port register data)
# 5 Polarity Inversion port 1
# 6 Configuration port 0 (configs the direction of the GPIO pins)
# 7 Configuration port 1
# '''
# cmd_byte = 0x00 # read port 0

DEVICE_BUS = 1
DEVICE_ADDR = 0x20
DEVICE_NB_GPIOS = 16

pca95xx = PCA95XX(DEVICE_BUS, DEVICE_ADDR, DEVICE_NB_GPIOS)



'''
All 16 I/O pins are input by default (mode 1)


Pins from 0 to 7 are the buttons (should be left as inputs)
They return with input() either 0 or 2^pinNb

Pins from 8 to 15 are ?? (for the noepixels we guess)
'''



def initPin(pinNb, mode):
    '''mode is "input" or "output"'''

    if mode == "input":
        pca95xx.config(pin=pinNb, mode=1)
        # print(f"Pin {pinNb} changed to input, res =  {pca95xx.config(pin=pinNb, mode=1)}")
    elif mode == "output":
        pca95xx.config(pin=pinNb, mode=0)
        # print(f"Pin {pinNb} changed to output, res = {pca95xx.config(pin=pinNb, mode=0)}")
    else:
        print("ERROR : Mode not supported")


def readPin(pinNb):
    res = pca95xx.input(pin=pinNb)
    # print(f"Pin {pinNb} read value : {res}")
    if res == 0: return 0
    else: return 1



def writePin(pinNb, value):
    pca95xx.output(pin=pinNb, value=value)
    # print(f"Pin {pinNb} set to output {value}, res = ", pca95xx.output(pin=pinNb, value=value))

