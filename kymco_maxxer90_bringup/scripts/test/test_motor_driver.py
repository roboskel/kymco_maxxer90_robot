#!/usr/bin/env python
import time
import serial
import serial.tools.list_ports

print("--FOUND PORTS--")
ports = list(serial.tools.list_ports.comports())
for p in ports:
    print(p)
print("---------------")

s2 = serial.Serial("/dev/ttyUSB1", 4800)
s1 = serial.Serial("/dev/ttyUSB2", 4800)

throttle_cmd = "$SPEED,1*1\r\n"
steering_cmd = "$ANGLE,25*1\r\n"

# Assuming s1 is the throttle board
# and s2 is the steering board

while(True):
    print("--SENDING THROTTLE COMMAND--")
    s1.write(throttle_cmd)
    print("--SENDING STEERING COMMAND--")
    s2.write(steering_cmd)

    print("----------- DONE -----------")
    print(s1.readline().strip())
    time.sleep(2)
