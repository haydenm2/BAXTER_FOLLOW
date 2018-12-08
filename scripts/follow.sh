#!/bin/bash
gnome-terminal -e "python identify_sticky.py"
gnome-terminal -e "rostopic echo /cam_data"
