from accelerometer import ADXL345
from giroscope import mpu6050

import Adafruit_SSD1306
from PIL import Image, ImageDraw,ImageFont

acc = ADXL345()
gyro = mpu6050(0x68)

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


while(True):
	
	# Draw a black filled box to clear the image.
    draw.rectangle((0,0,width,top+40), outline=0, fill=0)
	
    readings = {}
    readings['x'] = 0
    readings['y'] = 0
    readings['z'] = 0
    readings['xg'] = 0
    readings['yg'] = 0
    readings['zg'] = 0
    
    for i in range(10):
		dataB = acc.getAxes()
		dataA = gyro.get_gyro_data()
		readings['x'] += dataA['x']
		readings['y'] += dataA['y']
		readings['z'] += dataA['z']
    
    for axis in readings.keys():
		readings[axis] /= 10
		
    # Write four lines of text.

    draw.text((x, top),"Gyroscope data:",  font=font, fill=255)
    draw.text((x, top+8),"X: "+ str(format(readings['x'],'.2f')), font=font, fill=255)
    draw.text((x, top+16),Y: "+ str(format(readings['y'],'.2f')),  font=font, fill=255)
    draw.text((x, top+25),"Z: "+ str(format(readings['z'],'.2f')),  font=font, fill=255)
    
    disp.image(image)
    disp.display()
    
    print("\n\nLendo gyro:")
    print("x: ",format(readings['x'],'.2f'))
    print("y: ",format(readings['y'],'.2f'))
    print("z: ",format(readings['z'],'.2f'))
