import serial
import time
#import struct

ser = serial.Serial('/dev/ttyUSB0',115200 , timeout = 1) # ttyACM1 for Arduino board
out  = ''

while True:
	entrada = str(raw_input("Qual vel mudar? "))
	value = float(raw_input("Digite o valor: "))
	if entrada is "k":
		out = "A -234"
	if entrada is "a":
		out = "A " + str(int(value))
	if entrada is "l":
		out = "L "+ str(int(value))
		
	print(out + "\n")
	ser.write(out + "\n")
	ser.flush()	
