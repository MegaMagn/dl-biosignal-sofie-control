# Deep Learning-Based Biosignal Control of a Soft Hand Exoskeleton for Grasp Assistance

Soft, modular hand exoskeleton with integrated sensing and deep learning-based EEG/EMG control for grasp assistance.

## Overview

This repository contains the files used for the development of a soft hand exoskeleton and a deep learning-based biosignal classification framework for real-time grasp assistance.

The project is divided into two main parts:

* **Exoskeleton**
  Contains the mechanical design files, embedded firmware, electronics-related code, and supporting files required to fabricate, assemble, and control the soft hand exoskeleton.

* **Deep Learning-Based Classification**
  Contains the code used for EEG/EMG preprocessing, model training, validation, real-time classification, and biosignal-based control demonstrations.

Together, these components form a soft wearable exoskeleton system capable of assisting pinch grasping through tendon-driven actuation and user-intention decoding.

## Repository structure

```text
.
├── Deep Learning-Based Biosignal Control of a Soft Hand Exoskeleton for Grasp Assistance - report
├── Supplementary - Deep Learning-Based Biosignal Control of a Soft Hand Exoskeleton for Grasp Assistance
├── Exoskeleton
│   ├── Code
│   ├── Enclosure
│   ├── Index and thumb version
│   ├── Index only version
│   └── Miscellaneous
│
└── Deep Learning-Based Classification
    ├── 
    ├──
    ├──
    └──
```

## Folders and files

### Exoskeleton

#### Code

Contains the embedded and computer-side code used to control, communicate with, and log data from the exoskeleton.

* **Arduino_ESP32 project files**
  Contains the Arduino `.ino`, `.cpp`, and `.h` files for controlling the exoskeleton and logging sensor data on the ESP32.

* **Arduino_ESP32_EMG_control project files**
  Contains the Arduino `.ino`, `.cpp`, and `.h` files for controlling the exoskeleton using commands from the classification network.

* **ESPLogger.py**
  Python script for communication and data logging between the computer and the exoskeleton.

* **requirements.txt**
  Required Python packages for the exoskeleton-side logging and communication scripts.

#### Enclosure

Contains the `.stl` files used for the electronics and actuator enclosure.

#### Index and thumb version

Contains the `.stl` files for constructing the main glove, the thumb module, and the index module. This folder also includes different PTFE tube clamps used to secure the tendon guide tubes to the glove.

#### Index only version

Contains the `.stl` files for constructing the main glove and index finger module. This version was used for early testing and characterization of the integrated sensors.

#### Miscellaneous

Contains miscellaneous files used for test setups, fixtures, and supporting experiments.

### Deep Learning-Based Classification

Contains the code used to develop and evaluate the biosignal classification framework.

This includes scripts for:

* EEG and EMG preprocessing
* Dataset preparation and segmentation
* Subject-dependent and subject-independent model training
* LSTM, CNN+LSTM, and CNN+LSTM+Attention models
* EEG/EMG decision-level fusion
* Hyperparameter optimization
* Real-time inference
* Digital twin visualization
* Biosignal-based exoskeleton control demonstrations

## Hardware needed

The following hardware was used for the exoskeleton system:

* ESP32 30-pin development board
* Pololu Micro Metal Gear Motor, 6 V, 250:1 gear ratio
* Pololu 5 V step-up/step-down buck converter
* SparkFun Micro Magnetometer - MMC5983MA, Qwiic
* 6 mm × 2 mm round neodymium magnet
* Adafruit PCA9546 4-channel STEMMA QT / Qwiic I2C multiplexer
* Pololu DRV8835 dual motor driver
* 7.4 V, 1000 mAh, 2S LiPo battery
* 1 kΩ resistors for the StretchSense voltage divider
* Flexible braided thin steel wire for tendon actuation
* PTFE tubing for tendon guidance
* Assorted resistors, capacitors, and an operational amplifier for current sensing, if desired

## Materials and printing

The exoskeleton is primarily printed in **varioShore TPU** from colorFabb using a Prusa MK3S and a Prusa XL 5T.

Recommended printing settings from colorFabb were used, with a nozzle temperature and layer height of:

```text
220 °C
0.2 mm
```

The StretchSense sensor is printed using **Conductive Filaflex TPU** from Recreus. The filament should be dried before printing and printed using the recommended settings from Recreus. In this project, StretchSense was printed on a Prusa MK3S using a nozzle temperature and layer height of:

```text
240 °C
0.1 mm
```

## Software environment

The Python code was developed using Python 3.11.

Create and activate a virtual environment:

```bash
py -3.11 -m venv .venv
source .venv/Scripts/activate
```

Install the required Python packages:

```bash
pip install -r requirements.txt
```

On Windows PowerShell, the virtual environment can also be activated using:

```powershell
.venv\Scripts\Activate.ps1
```

## Arduino / ESP32 setup

The exoskeleton firmware is intended to run on an ESP32 development board using the Arduino framework.

Before uploading the firmware, make sure that:

* The ESP32 board package is installed in the Arduino IDE.
* The correct ESP32 board and COM port are selected.
* Required libraries for the motor driver, I2C sensors, and communication are installed.
* The wiring matches the pin definitions used in the firmware configuration files.

## Notes

This repository contains research prototype files. The exoskeleton was developed for experimental validation and should be tested carefully before being used with human participants.

When working with the device:

* Check tendon routing before actuation.
* Verify motor direction and software limits.
* Ensure that the exoskeleton can release tension safely.
* Avoid excessive tendon tension.
* Disconnect power when modifying the electronics or mechanical structure.

## Project title

**Deep Learning-Based Biosignal Control of a Soft Hand Exoskeleton for Grasp Assistance**

## Authors

Magnus Malthe Sigsgaard Nielsen  
Nicklas Nikolaj Grønvall

University of Southern Denmark
SDU Biorobotics / SDU Soft Robotics

## License

## License

No license has currently been selected for this repository. All rights reserved.

The contents of this repository are made available for reference and documentation purposes only. Non-commercial use, sharing, distribution, and reproduction in any medium or format are permitted, provided that appropriate credit is given to the original authors and source
