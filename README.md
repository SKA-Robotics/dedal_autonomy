# **dedal_autonomy Repository**
This repository contains software from the **DEDAL** drone autonomy.


## Requirements
TODO

## How to use this repository
Clone repository to Dedal main computer and copy **ws_controller** folder to main directory.

```bash
git clone https://github.com/SKA-Robotics/dedal_autonomy.git
cp -r dedal_autonomy/ws_controller ~/
``` 
## How to lunch main programs
Every single program **must** be run in different console and be active all the time:
1. To brodcast connection to flight controller run:
```bash
./proxy.sh
```
2. To read data and send connands to flight controller run:
```bash
ros2 run flight_controller_pkg fc_controller
```
3. To run Flask interface run:
```bash
ros2 run flask_pkg flask_server
```
