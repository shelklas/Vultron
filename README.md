# Vultron

Fully integrated Flight Management System. Allows for automated flight guidance following a given flight plan. 

## Getting Started
 Input for the flight are given through the `main.cpp`. There is no limit to how large your flight plan is. Due to the accuracy of civilian GPS modules, you MUST have the waypoints at least `10` meters apart from one another.
 
### Dependencies

* [GPSD](http://www.catb.org/gpsd/) - GPS Framework used
* [Adafruit BNO055](https://github.com/adafruit/Adafruit_BNO055) - 9 DOF Orientation Sensor Driver

### Installation
The following programs must be installed:
```
sudo apt-get install gpsd gpsd-clients
```

#### Assumptions
The program assumes the GPS is correctly installed on pin `/dev/ttyAMA0`. If this is not done,
the program will NOT be able find the GPS module. You may use any GPS that is able to be 
processed by GPSD.
