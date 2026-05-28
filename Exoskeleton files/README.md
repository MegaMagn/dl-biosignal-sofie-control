# SoFiE: Soft Finger Exoskeleton for Intelligent Grasping
Soft, wearable, 3D printable, and modular exoskeleton with integrated tactile and proprioceptive sensing.

## Overview
Here, you will find all the necessary files to print and assemble your own soft exoskeleton and control it.

## Folders and files
- Code  
  - Arduino_ESP32 project files  
      Contains the Arduino .ino file and the .cpp and .h files for controlling and logging data on the ESP32.
  - Arduino_ESP32_EMG_control project files  
      Contains the Arduino .ino file and the .cpp and .h files for controlling the exoskeleton with the classification network.
  - ESPLogger.py  
      The logging and communication script meant to run on the computer in conjunction with the exoskeleton.
  - requirements.txt  
      required python packages
- Enclosure  
    Contains the .stl files used for the electronics enclosure.
- Index and thumb version  
    Contains the .stl files for constructing the main glove part, the thumb module and the index module, along with an assortment of PTFE tube clamps used to secure the PTFE tubes to the glove 
- Index only version  
    Contains the .stl files for constructing the main glove part and the index module.
- Miscellaneous  
    Contains some miscellaneous files for some of the test setups
   
## Hardware needed
* ESP32s 30 pin devboard
* Pololu Micro Metal Gear Motor (6V, 250:1)
* Pololu 5V Step up/Step down buck converter
* sparkFun Micro Magnetometer - MMC5983MA (Qwiic)
* 6 mm x 2 mm round Neudymium magnet
* Adafruit PCA9546 4-Channel STEMMA QT / Qwiic I2C Multiplexer
* Pololu DRV8835 Dual Motor driver
* 7.4 V, 1000 mAh, 2S LiPo battery
* 1 k Ohm resistors for StretchSense voltage divider
* flexible braided thin steel wire
* A lenght of PTFE tubing
* Assortment of resistors, capacitors and an opAmp for current sense if wanted.

## Materials and printing
The exoskeleton is primarily printed on a Prusa MK3s and a Prusa XL 5T in red varioShore TPU from Colorfabb. It is printed using their recommended settings, at a nozzle temperature of 220°C  
The StretchSense sensor is made of Conductive Filaflex TPU from Recreus, again dried and printed using their recommended settigns on a Prusa MK3s at a nozzle temeperature of 240°C

## Environment
Python version: 3.11  
Install:  
py -3.11 -m venv .venv  
source .venv/Scripts/activate  
pip install -r requirements.txt
