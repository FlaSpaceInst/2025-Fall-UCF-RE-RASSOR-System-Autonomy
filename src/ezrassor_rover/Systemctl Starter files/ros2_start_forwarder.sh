#!/bin/bash


sleep 10

# Second Source
source /root/hardware_ws/install/setup.bash
ros2 run rassor_serial_fwd forward

# Done

