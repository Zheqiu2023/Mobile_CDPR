#!/bin/bash

echo ""
echo "This script copies a udev rule to /etc to facilitate bringing"
echo "up the cdpr usb connection."
echo ""

sudo cp `rospack find cdpr_bringup`/scripts/70-ttyusb.rules /etc/udev/rules.d/

echo ""
echo "Restarting udev"
echo ""

sudo service udev reload
sudo service udev restart

echo "Finish "