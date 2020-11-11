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

start_s = "$ANGLE,"
steering_cmds = [0,25,49,25]
end_s = "*11\r\n"
start_t = "$SPEED,"
throttle_cmds = [0,1,2,3,4,5,6,7,8,9,8,7,6,5,4,3,2,1]
end_t = "*11\r\n"

# Assuming s1 is the throttle board
# and s2 is the steering board

while(True):
    for i in throttle_cmds:
        print("--SENDING THROTTLE COMMAND--")
        s1.write(start_t+str(i)+end_t)
        #  print("----------- READING -----------")
        #  print(s1.readline().strip())
        time.sleep(2)
    for i in steering_cmds:
        print("--SENDING STEERING COMMAND--")
        s2.write(start_s+str(i)+end_s)
        #  print("----------- READING -----------")
        #  print(s2.readline().strip())
        time.sleep(2)
