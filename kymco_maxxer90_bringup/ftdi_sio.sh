#!/bin/sh
chmod 777 /sys/bus/usb-serial/drivers/ftdi_sio/new_id && echo "0403 bd90" > /sys/bus/usb-serial/drivers/ftdi_sio/new_id

