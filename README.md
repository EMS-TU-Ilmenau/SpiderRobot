# Spider robot

Controller for a spider-cam-like positioner (cable suspended robot).

### Author
Niklas Beuster -- niklas.beuster@tu-ilmenau.de

## Description
This project provides a system to build the presentation of a cable suspended robot with arbitrary cables.
It cares about the calculation of the cable lengths and rotation speeds for each winch when moving the platform to a new target position.
The motors need a serial interface to send them rotation rate and rotation position at minimum.

## How to install
Clone this repository somewhere to the host computer that will control your devices.
In the newly created directory, install the package with: `pip install .`

Now, you can import **spiderrobot** from anywhere on that computer in python.

## Usage
1. Define a coordinate system in the real world for the application, i.e. where the target is moving in and the motor axes are placed.
2. Instantiate the [Positioner](spiderrobot/positioner) class with the serial interface for the motor controller and the initial target position.
3. Add motor abstractions using the "addAxis" method with the motor id, position of the axis and the axle diameter in meters.
4. Move the target using the "moveToPos([x, y, z], velocity)" method, with argumements being metric.