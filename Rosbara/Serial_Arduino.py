import serial
import time
#import struct

ser = serial.Serial('COM8',115200 , timeout = 0.1) # ttyACM1 for Arduino board
readings = []
command = ""
time.sleep(3)
def send_vel(linear, angular):
    ser.write("L " + str(linear) + "\n")
    ser.write("A " + str(angular) + "\n")

def handle_input():
    global command
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

for i in range(10):
        time.sleep(0.1)
        handle_input()
        print('Bora comecar!')
        send_vel(i,2*i)
        print(readings)
while(ser.in_waiting):
    handle_input()
    print(readings)
