#!/bin/bash

# Shell script file
# Created by Orion Schyberg for Florida Space Institute
# MIT Licensed, please see the license information at https://github.com/Monstarules/rassor_serial_forward

# 15s delay in case of delay in startup
sleep 10

# First Source
source /root/software_ws/install/setup.bash
ros2 run ezrassor_controller_server controller_server

# Done
