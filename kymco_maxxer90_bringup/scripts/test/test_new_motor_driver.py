#!/usr/bin/env python
import time
import serial
import serial.tools.list_ports

print("--FOUND PORTS--")
ports = list(serial.tools.list_ports.comports())
for p in ports:
    print(p)
print("---------------")

s1 = serial.Serial("/dev/ttyUSB0", 57600)

start_s = "CSL"
start_s2 = "CSR"
steering_cmds = [3000,2000,500]
end_s = ""
start_t = "CSI"
start_t2 = "CSD"
throttle_cmds = [500,1000,2000]
end_t = ""

while(True):
    for i in throttle_cmds:
        print("--SENDING THROTTLE COMMAND--")
        command = start_t+str(i)+end_t
        print(command)
        s1.write(command)
        #print("----------- READING -----------")
        #print(s1.readline().strip())
        #time.sleep(2)
    for i in throttle_cmds:
        print("--SENDING THROTTLE COMMAND--")
        command = start_t2+str(i)+end_t
        print(command)
        s1.write(command)
        #  print("----------- READING -----------")
        #  print(s1.readline().strip())
        #time.sleep(2)
    for i in steering_cmds:
        print("--SENDING STEERING COMMAND--")
        command = start_s+str(i)+end_s
        print(command)
        s1.write(command)
        #  print("----------- READING -----------")
        #  print(s2.readline().strip())
        #time.sleep(2)
    for i in steering_cmds:
        print("--SENDING STEERING COMMAND--")
        command = start_s2+str(i)+end_s
        print(command)
        s1.write(command)
        #  print("----------- READING -----------")
        #  print(s2.readline().strip())
        #time.sleep(2)
