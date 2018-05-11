# Introduction

Dynamixels are servomotors with a working angle of 300° dispatched on values 0 to 1023.

![alt text](electronics/actuators/Dynamixels_SRC/AngleFonctionnement.png )

They also have the particularity of beeing able to be used as DC motor. They have a very good motor torque what makes them efficient for a big number of applications. In our robots, we used them for the tasks the robots had to realise during the competition as : 

*	Sort balls of different colors
*	Deploy an arm to push an object on wheels
*	Deploy a platform on multiple floors to liberate cubic blocks
*	Maintaining the blocks on every floor or not
*	Actionate a gear to move blocks

The cabling is made with JST 3 pin connectors from a servomotor to another.

Here is a link on amazon to order some : https://www.amazon.fr/ensembles-Micro-connecteur-Fiche-150mm/dp/B01DU9OY40/ref=sr_1_2?ie=UTF8&qid=1525997056&sr=8-2&keywords=jst+connecteur+3+pin&dpID=51oVa4jux4L&preST=_SY300_QL70_&dpSrc=srch

 ![alt text](electronics/actuators/Dynamixels_SRC/PinsConnections.png )

The Dynamixels have 2 locations for these connectors because it is possible to connect several servomotors in series and to control them with an addressing. These adresses are represented by unique ID for every dynamixel and it is possible to check with the software Dynamixel Wizard of Roboplus and a USB2Dynamixel.

![alt text](electronics/actuators/Dynamixels_SRC/USB2Dynamixel.png )

Thanks to this software, it is also possible to configurate the dynamixels (registers) like, among others, the baudrate of the servomotor and also to access a serie of informations in real time like the speed, the position, etc.
