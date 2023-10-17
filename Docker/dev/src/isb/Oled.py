# -*- coding: utf-8 -*-

from time import sleep

import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

from numpy import linspace





# Load default font.
font = ImageFont.load_default()

#font = ImageFont.truetype('/usr/share/fonts/truetype/freefont/FreeMono.ttf', 15, encoding="unic")

class Oled():

    def __init__(self):

        # parameter, 3 characters max
        self.signs = ["BR", "ACT", "LID", "LCD", "CAM"]


        self.nb_signs = len(self.signs)

        # 0 : unknown/problem | 1 : OK
        self.sign_states = [0]*5


        self.sign_coords = linspace(0, 127, self.nb_signs+1).astype(int)
        self.cpu_coords = linspace(0, 50, 5).astype(int)


        self.CPULoads = [0]*4
        self.CPUTemp = 0

        # 128x32 display with hardware I2C:
        self.disp = Adafruit_SSD1306.SSD1306_128_64(rst=None)

        # Initialize library.
        self.disp.begin()

        # Clear display.
        self.disp.clear()
        self.disp.display()

        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        self.width = self.disp.width
        self.height = self.disp.height
        self.image = Image.new('1', (self.width, self.height))

        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(self.image)

        # Draw a black filled box to clear the image.
        self.draw.rectangle((0,0,self.width,self.height), outline=0, fill=0)

        # Draw some shapes.
        # First define some constants to allow easy resizing of shapes.
        padding = -2
        self.top = padding
        self.bottom = self.height-padding
        # Move left to right keeping track of the current x position for drawing shapes.
        self.x = 0


    def oled_display_string(self, str, x, y):

        # clear
        self.draw.rectangle((0,0,self.width,self.height), outline=0, fill=0)

        self.draw.text((x, y), str, font=font, fill=255)

        self.disp.image(self.image)
        self.disp.display()


        return

        # Draw a black filled box to clear the image.
        self.draw.rectangle((0,0,self.self.width,self.self.height), outline=0, fill=0)

        # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
        cmd = "hostname -I | cut -d\' \' -f1"
        IP = subprocess.check_output(cmd, shell = True )
        cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
        CPU = subprocess.check_output(cmd, shell = True )
        cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
        MemUsage = subprocess.check_output(cmd, shell = True )
        cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
        Disk = subprocess.check_output(cmd, shell = True )

        # Write two lines of text.

        self.draw.text((x, top),       "IP: " + str(IP),  font=font, fill=255)
        self.draw.text((x, top+8),     str(CPU), font=font, fill=255)
        self.draw.text((x, top+16),    str(MemUsage),  font=font, fill=255)
        self.draw.text((x, top+25),    str(Disk),  font=font, fill=255)

        # Display image.
        self.disp.image(image)
        self.disp.display()
        time.sleep(.1)


    # def display_image(self, fileName, x, y, w, h):

    #     self.bg = Image.open(fileName).convert('1')


    def set_bgImage(self, fileName):

        self.bg = Image.open(fileName).convert('1')
        self.disp.image(self.bg)
        self.disp.display()


    def update_connections(self):

        id_vendors = [0,0,0,0,0]  # TODO : a def

        usb_info = subprocess.check_output("lsusb", shell = True)

        usb_info = str(usb_info).split('ID ')[1:]
        nb_usb_info = len(usb_info)

        for k in range(nb_usb_info):

            print(usb_info[k][0:9])

            self.sign_states[k]



    def update_stats(self):

        '''
        CPU usage for each CPU (bar diagram ?)
        Memory usage
        CPU temperature
        '''

        cmd = "top -b1 -n1 | grep Cpu"
        cpu_info = subprocess.check_output(cmd, shell = True)
        cpu_info = str(cpu_info).split('ni')
        for k in range(1, len(cpu_info)):
            self.CPULoads[k-1] = 100 - float(cpu_info[k][1:6])
        

        cmd = "cat /sys/class/thermal/thermal_zone0/temp"
        self.CPUTemp = round(float(subprocess.check_output(cmd, shell = True))/1000, 1)



    def update_display(self):

        # top bar
        for k in range(self.nb_signs):

            # TODO : display constant, depends on the number of signs
            offset_chars_list = [1, 4, 7, 11]

            if self.sign_states[k] == 1:
                self.draw.rectangle((self.sign_coords[k]+1, 0, self.sign_coords[k+1]-1, 15), outline=0, fill=1)

                x = (self.sign_coords[k] + self.sign_coords[k+1]) // 2 - offset_chars_list[len(self.signs[k])-1]
                self.draw.text((x, 2), self.signs[k], font=font, fill=0)
                
            else:
                self.draw.rectangle((self.sign_coords[k]+1, 0, self.sign_coords[k+1]-1, 15), outline=0, fill=0)
        
                x = (self.sign_coords[k] + self.sign_coords[k+1]) // 2 - offset_chars_list[len(self.signs[k])-1]
                self.draw.text((x, 2), self.signs[k], font=font, fill=1)


        # clear lower section
        self.draw.rectangle((0, 16, 127, 63), outline=0, fill=0)

        # display CPU bar graph
        for k in range(4):
            self.draw.rectangle((self.cpu_coords[k]+2, 48 - 32*self.CPULoads[k]/100, self.cpu_coords[k+1]-2, 50), outline=0, fill=1)
        
        # TODO : draw only once ?
        self.draw.text((18, 16), "CPU", font=font, fill=1)
        self.draw.text((12, 55), "{0:.2f}".format(sum(self.CPULoads)), font=font, fill=1)
        self.draw.text((70, 25), "MEM", font=font, fill=1)
        self.draw.text((64, 45), "TEMP", font=font, fill=1)

        self.draw.text((98, 45), str(self.CPUTemp)+'°', font=font, fill=1)



        self.disp.image(self.image)
        self.disp.display()


    def run(self):

        while True:

            self.update_stats()
            self.update_display()


            sleep(0.8)
        

if __name__ == '__main__':


    '''
    Pixels going from (0,0) to (127,63)
    The upper band goes from (0,0) to (127,15)
    '''

    oled = Oled()

    # oled.set_bgImage('SRC_OledLogo2.ppm')

    oled.run()
    
    # oled.update_display()
    # sleep(0.2)
    # oled.update_display()
    # sleep(0.2)
    # oled.update_display()
    # sleep(0.2)

    # oled.sign_states[2] = 1

    # oled.update_display()
    # sleep(0.2)    
    # oled.update_display()
    # sleep(0.2)
