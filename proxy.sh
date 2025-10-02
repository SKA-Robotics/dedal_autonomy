#! /bin/bash
mavproxy.py --baudrate=57600 --master=/dev/ttyACM0 \
 --out udp:0.0.0.0:14550 \
 --out udp:aero.local:14550 #Tu musi być IP laptopa robiącego za GS
