#!/usr/bin/env python
import serial
from time import sleep
import serial.tools.list_ports

print("--FOUND PORTS--")
ports = list(serial.tools.list_ports.comports())
for p in ports:
    print(p)
print("---------------")

s1 = serial.Serial("/dev/ttyACM0", 19200)

start_cmd = "CV0"
stop_cmd  = "CV1"

# Assuming s1 is the throttle board
# and s2 is the steering board

print("--SENDING START COMMAND--")
s1.write(start_cmd)
sleep(2)
print("--SENDING STOP  COMMAND--")
s1.write(stop_cmd)

print("---------- DONE ---------")

print("--READING ENCODERS FOREVER--")
while(True):
    print(s1.readline().strip())

