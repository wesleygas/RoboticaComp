import serial
import time
#import struct

ser = serial.Serial('/dev/ttyS0',115200 , timeout = 1) # ttyACM1 for Arduino board
data=5.724
print('galera')

for i in range(10):
        print('de')
        ser.write(str(data*i))
        time.sleep(2)
        print(data*i)
        ser.flush()
