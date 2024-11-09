# Robot-Controller

In this repository, I programmed a controller for a Lego [SPIKE](https://spike.legoeducation.com/) robot that contained two motors for two wheels, a gyroscope, and an ultrasonic sensor. 
Those sensors are connected to the robot via a set of ports, and whose data was received using the API of the SDK that is set up for the robot. So that meant that all that was left was handling the control logic. 

The primary function of the controller is to use PID control to maintain the direction pointed by the robot when it is turned on. So when it is pushed to the side or turned around, it readjusts. 
