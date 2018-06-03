#!/usr/bin/python
import Adafruit_SSD1306
from PIL import Image, ImageDraw,ImageFont
import commands
import time
import os

time.sleep(20)

RST = None #Muda a variavel dentro da biblioteca pro LCD
disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)

disp.begin()

disp.clear() #Marca tudo para ser limpo
disp.display()#Manda pro display

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

padding = -2
top = padding
bottom = height-padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0

# Load default font.
font = ImageFont.load_default()

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,top+40), outline=0, fill=0)

ssid = os.popen("iwconfig wlan0 \
                | grep 'ESSID' \
                | awk '{print $4}' \
                | awk -F\\\" '{print $2}'").read()

draw.text((x, top),"Meu IP:",  font=font, fill=255)
draw.text((x, top+8),"IP: "+ str(commands.getoutput('hostname -I')), font=font, fill=255)
draw.text((x, top+16), "SSID: "+ str(ssid), font=font,fill = 255)
disp.image(image)
disp.display()
