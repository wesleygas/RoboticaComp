import serial
import time
import numpy as np
#import struct

ser = serial.Serial('COM12',115200 , timeout = 0.1) # ttyACM1 for Arduino board
readings = []
command = ""
time.sleep(3)
def send_vel(linear, angular):
    ser.write("L " + str(linear) + "\n")
    ser.write("A " + str(angular) + "\n")

def handle_input():
    global command
    global readings
    if (ser.in_waiting):
        data = str(ser.read()) #Le byte por byte
        if(data == "\n"): #Se instrucao acabou, parse it
            if(command[0] == "U"): #Se tem HEAD
                command = command[2:] #Exclui Head
                command = command.split(",")[0:-1] #Exclui ultima entry
                if(len(command) == 3): #Checa integridade
                    readings.append(command)
                    if(len(readings) > 10):
                        del readings[0] #Mantem queue size 10
            command = ""
        else: #Adiciona cada byte no buffer
            command += data


print('galera')

while(True):
    for i in range(4):
        handle_input()
    if(len(readings)>0):
        print(readings[-1])
    time.sleep(0.1)
