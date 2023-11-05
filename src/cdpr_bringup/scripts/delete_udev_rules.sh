#!/bin/bash
echo "Delete a udev rule of cdpr usb connection in /etc"

echo "Delete udev in the /etc/udev/rules.d/"

sudo rm   /etc/udev/rules.d/70-ttyusb.rules

echo " "
echo "Restarting udev"
echo ""

sudo service udev reload
sudo service udev restart

echo "Finish"