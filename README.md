# Airship Simulation

This project aims to build a 3D simulation of the airship and the environment it will be flying in for the 2020 GDP Airship project. The airship is able to be flow manually or autonomously.



## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

This project was built in **[MATLAB_2020a](https://www.mathworks.com/downloads/web_downloads/download_release?release=R2020a)**. The following packages are also required to run the simulation

- [Simulink Toolbox](https://www.mathworks.com/products/simulink.html)
- [Robotics System Toolbox](https://www.mathworks.com/products/robotics.html)
- [3D Animation Toolbox](https://www.mathworks.com/products/3d-animation.html)

### Installing

To download the repo, run the following commands

```bash
mkdir GDP
cd GDP
git clone https://github.com/ic-aero-airship-2020/lidarscansim.git .
```



## Running the simulation

The simulation can be run from the script **FullSimulationRun.m**



## General Notes

- Formatted as a 3xN cell matrix. first row is the data for the front sensor, second row is for the left sensor and third row is for the right sensor
- Sensor is set as having a range of 10m, field of vision of 20deg and positions at 2,6 and 10 o'clock as agreed in meeting on 15.05.2020



## FATT02 Notes

- 

## FATT05 Notes

- 

## FATT06 Notes

- 