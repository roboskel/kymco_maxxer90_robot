#!/bin/sh
sudo chmod 777 /sys/bus/usb-serial/drivers/ftdi_sio/new_id && sudo echo "0403 bd90" > /sys/bus/usb-serial/drivers/ftdi_sio/new_id

