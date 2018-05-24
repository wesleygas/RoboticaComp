
import smbus
import time

bus = smbus.SMBus(1)

#Arduino address
address = 0x04

def writeI2C(data):
	bus.write_byte(address,data)

def readI2C():
	return bus.read_byte(address)

while True:
	dados = input("Manda um numero ae: ")
	
	writeI2C(dados)
	
	print("Blz, to mandando isso: ", dados)
	
	time.sleep(0.5)
	
	retorno = readI2C()
	
	print("E recebi isso dele:", retorno)
