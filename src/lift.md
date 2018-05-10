# Lift 

To raise the coloured blocks up and sort them, they are first pushed into a lift. 
To do so we use DC motors to collect the blocks inside the robot. 
These motors are used like rollers in a car wash to suck in the blocks. 
To detect if a block is close enough to the lift we use a limit switch (see picture below). 

![img](img/mechanical/minus/lift/limit_switch.png)

It is connected to a pin of an Arduino Uno board so we can detect the state change when a block touches the switch. 
That event triggers the lift and thus the block starts to raise up. 
Therefore we are using another DC motor with a belt (like the one you can see on the picture below). 

![img](img/mechanical/minus/lift/belt.png)

Thanks to the colour detection we know in which order we have to collect the blocks on the field, so we also know which one comes first in the lift and which floor it has to go to. 
It is important to know the specific floor in order to create the pattern showed at the beginning of the game and recognised by the colour and sequence recognition part.

To know when the lift has to stop, we use the encoder of the DC motor so we can precisely know when the motor has to stop. 
The encoder is connected to an interruption pin on the Arduino board so the tick counts can be more accurate and have priority.

Once the block is on the right floor, it has to be pushed outside of the lift to already be placed in its position on the tower on a kind of shelf (as you can see in the picture below) which supports the blocks even if there is nothing below. 

![img](img/mechanical/minus/lift/shelf.png)

Furthermore, to push the block, we use a dynamixel with a gear and a tooth rack to slide a plate along which pushes the block until it reaches the « shelf ». 

Then the lift (without any block inside) goes down. To make sure it doesn’t go too much we use another limit switch. 
Therefore the lift is immediately stopped, another cube comes in and we can do the cube lifting steps mentioned before over again until all the cubes of the tower are placed. 


As mentioned above, we are using DC motors to control our lift. 
We have in all 3 motors and they are all connected to a driver (see picture below) controlled by an Arduino Uno board. 

![img](img/mechanical/minus/lift/driver.png)

We use 2 motors for the rollers to collect the blocks. In order to do so they need to turn in the opposite direction but at the same speed. 
Thus we connected them to the same driver but inverted the pins of one motor compared to the other one. 
These motors don’t have an encoder so their only 2 pins are connected to the driver. We also have the third motor to make the lift go up and down. As we said before, we use the encoder to know where have to stop so there are pins of the motor connected to the Arduino board. On the other hand the motor is also connected to the driver so we can control its speed, stop it and launch it.



## References for the components of the lift
* *IGUS rail*
  * Small one
    * Carriage : Igus Linear Guide Carriage NW-02-17, N
  * Big one : 
    * Rail : Igus N Series, NS-01-27-300, Linear Guide Rail 27mm width 300mm length
    * Carriage : Igus Linear Guide Carriage NW-02-27, N
* *Lift transmission*
  * Pitch (distance between teeth) : 2.032 mm
  * Pulley : aluminium, Glass Filled PC Timing Belt Pulley, 6mm Belt Width x 2.032mm Pitch, 36 Tooth, Maximum Bore Dia. 5mm
  * Timing belt : RS Pro, Timing Belt, 315 Tooth, 640.08mm Length X 6mm Width
