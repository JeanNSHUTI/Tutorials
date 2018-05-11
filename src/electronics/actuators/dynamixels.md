# Specifications

*	Weight : 53.5g (AX-12/AX-12+), 54.6g (AX-12A)
*	Dimension : 32mm * 50mm * 40mm
*	Resolution : 0.29°
*	Gear Reduction Ratio :  254 : 1
*	Stall Torque : 1.5N.m (at 12.0V, 1.5A)
*	No load speed : 59rpm (at 12V)
*	Running Degree :  0° ~ 300° or Endless Turn
*	Running Temperature : -5℃ ~ +70℃
*	Voltage : 9  ~ 12V (Recommended Voltage 11.1V)
*	Command Signal : Digital Packet
*	Protocol Type : Half duplex Asynchronous Serial Communication (8bit,1stop,No Parity)
* Link (Physical) : TTL Level Multi Drop (daisy chain type Connector)
*	ID : 254 ID (0~253)
*	Communication Speed : 7343bps ~ 1 Mbps
*	Feedback : Position, Temperature, Load, Input Voltage, etc.
*	Material : Engineering Plastic


# Dynamixel Wizard

*	Download and install the software "Roboplus" on the website www.robotis.com when going in Support>Download>Software>Roboplus
 
 ![alt text](electronics/actuators/Dynamixels_SRC/DW_Roboplus.png )
 
*	Launch the software and clic on Dynamixel Wizard 

 ![alt text](electronics/actuators/Dynamixels_SRC/DW_DynamixelWizard.png )

*	Insert the USB2Dynamixel in the USB port of the computer 
*	Connect the 3 pins JST connector in the USB2Dynamixel (TTL side) ansd the dynamixel to verifie or configure
*	Select « TTL » with the switch on the USB2Dynamixel
*	Supply the dynamixel separately in its working voltage (9-12V, recommanded 11,1V) with the second port of the dynamixel

 ![alt text](electronics/actuators/Dynamixels_SRC/DW_USB2Dynamixel.png )

*	Select the port of your computer where the USB2Dynamixel is connected

 ![alt text](electronics/actuators/Dynamixels_SRC/DW_Port.png )

*	Clic on "Open Port"

![alt text](electronics/actuators/Dynamixels_SRC/DW_OuvrirLePort.png ) 

* Make a basic research of the dynamixel to find its ID


*	It is also possible to make advanced research on other baudrates in case of the basic research is not working

*	Once the dynamixel found, selct it on the left side of the window 

*	The details of the informations of the dynamixel appears

*	in these details, it is possible to modify the configuration of the dynamixel like its ID, its communication speed (baudrate), its working speed, etc, or also to control it in real time.


# Examples of use

To use the dynamixels, it is necessary to download first the library "AX_12A_servo_library" that you can find on the github of eurobot here : https://github.com/Ecam-Eurobot/Tutorials/tree/master/src/electronics/actuators/Dynamixels_SRC

You'll have to extract it and place it in Documents/Arduino/libraries.

To configure the dynamixels, we'll use the Arduino software app.

This library possess 4 exemples of use of the dynamixel : 
-	Blink : example of control of the red led you can find in the back of every dynamixel
-	Move : example of control of the dynamixel in servomotor mode by giving it a position
-	EndlessTurn : example of use in DC motor mode (or wheel mode)
-	ReadRegister : usefull to dispaly the contents of the registers of the dynamixel

These examples are located in Fichier>Exemples>AX-12A on the Arduino app on your computer.

After selecting one of the examples, you have to include the library AX-12A without declaring a specific path because the library is located in the fold "libraries" of the fold "Arduino".

 ![alt text](electronics/actuators/Dynamixels_SRC/Exemples_Include.png )

After that you have to configure 3 lines in the code that respond to the needs of the configuration of the connected dynamixel you want to configure :

```
#define DirectionPin  (10u)
#define BaudRate      (1000000ul)
#define ID            (1u)
```

"DirectionPin" is used to indicate the communication direction of the dynamixel. 10u is for writing in the registers so you don't have to change the value.

"BaudRate" defines the communication speed used ( and configurate before with the Dynamixel Wizard)

"ID" represents the ID of the dynamixel that you checked or configured before for the addressing.

# References

-	e-manual : http://support.robotis.com/en/ 

-	youtube vidéo for all the steps of the use of the Dynamixel Wizard : https://www.youtube.com/watch?v=YJ9b68hx5Qc&version=3&hl=ko_KR

-	website of the manufacturer : http://en.robotis.com

- AX-12A library : https://github.com/ThingType/AX-12A-servo-library

