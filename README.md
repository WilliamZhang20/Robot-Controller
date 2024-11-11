# Robot-Controller

In this repository, I programmed a controller for a Lego [SPIKE](https://spike.legoeducation.com/) robot that contained two motors for two wheels, a gyroscope, and an ultrasonic sensor. 
Those sensors are connected to the robot via a set of ports, and whose data was received using the API of the Lego Education Spike SDK set up for the robot. With all the hardware set up, all that was left was handling the control logic. 
The robot has a USB cable connection to my PC. I open up the app, write the code, and since the app detects the robot's connection, I press 'download code to hub' and it will load the code into the robot. 

## PID Control Mechanisms

The controller's primary function is to use PID control to maintain the direction pointed by the robot when it is turned on. So when it is pushed to the side or turned around, it readjusts. 
Proportional-integral-derivative (PID) is a feedback control loop mechanism to automatically adjust an input to a desired setting, also known as a set point (SP). The variable that is fed back is the actual value of the system, or the process variable (PV). The controller continuously computes the difference between the PV and SP to obtain an error value, which is used in the calculation of a control variable that is fed into the control plant. This calculation, sums up three terms, P, I, and D. 

1. The P term takes the error and multiplies it by a constant Kp. If it's small, less adjustment is needed. If it's big, more adjustment is needed. Pretty intuitive, but not very perfect.
2. The I term takes the error, integrates it continuously, and multiples it by a constant Ki. This corrects effects of past error transient effects that result from the P term being so small that the error barely changes. However, this type of control process is also the most sensitive. 
3. The D term takes the derivative of the error and multiples it by a constant called Kd. This tends to correct for noise in the sensor or fast disturbances in the input, or in other words, any future errors. 

In the course correction, the robot takes an initial measurement of the gyroscope measurement when it is turned on, and sets that to be the set point. If the robot is pushed to point away from that direction, it should automatically turn its wheels to point back to the original direction. After extensive tuning via brute force trial and error, I determined a correct set of PID constants that ensured stable error correction that minimized oscillations and overshoots. 

## Further add-ons

I added two features to the controller in addition to the raw PID for course correction. The first was anti-windup clamping to the integrator. This is to prevent the integrator from accumulating too much, which would cause the control plant to saturate. 

The second addition was the use of a [feedforward](https://web.stanford.edu/class/archive/ee/ee392m/ee392m.1034/Lecture5_Feedfrwrd.pdf) adjustment that adds a multiple of the set point to the controller output, turning it into a _hybrid_ control system. Feedforward action enables correction to be taken earlier before disturbances arrive at the output. Overall, the effect was very noticeable and resulted in a very smooth correction, as seen in the video attached to the repository. 

Finally, I used the ultrasonic sensor of the robot and added a layer of PID to control the speed output of the robot such that the setpoint of the distance in front of the robot is 10 centimetres. However, if the robot comes too close to an object, all previous control outputs are overridden and the robot stops itself.
