#!/bin/bash


PYTHON_FILE1="/home/kakaping/C_library/teachur5_code/src/locatornew/scripts/block_detector.py"
PYTHON_FILE2="/home/kakaping/C_library/teachur5_code/src/locatornew/scripts/hw_grab.py"
PYTHON_FILE3="/home/kakaping/C_library/teachur5_code/src/dual_arm/manipulator/scripts/hw_move.py"

gnome-terminal -- bash -c "python3 $PYTHON_FILE1; exec bash"
sleep 2

gnome-terminal -- bash -c "python3 $PYTHON_FILE2; exec bash"
sleep 2

gnome-terminal -- bash -c "python3 $PYTHON_FILE3; exec bash"

