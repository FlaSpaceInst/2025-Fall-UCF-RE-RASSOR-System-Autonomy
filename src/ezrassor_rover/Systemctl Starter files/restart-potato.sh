#!/bin/bash

# This script will restart the potato-discovery service when the potato turns on
# Below are where you move this script and the command to make it executable

#sudo nano /lib/systemd/system-sleep/restart-potato.sh
#sudo chmod +x /lib/systemd/system-sleep/restart-potato.sh

case "$1" in
  post)
    echo "Resuming from sleep, restarting potato-discovery.service..." >> /var/log/potato-resume.log
    systemctl restart potato-discovery.service
    ;;
esac
