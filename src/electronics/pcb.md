# Printed Circuit Boards

>*last updated on May 10, 2018*
> 

Three types of printed circuit boards were developed for our two robots. The following section presents these boards, it utilities and it conception 
and gives you all the necessary elements to understand, recreate and use it. 

## Starting Board
Inspired by past version of the starting board

The utility of this PCB is to gather the component serving to the start sequence. Normally this is the only interface with the robot during the 
competition.

First, we used three switches for configuring three main points of our robot : 

* Team configuration
We have to setup the starting side for the next match to the robot. It behaviour is different if the robot play as the green or the orange team. 
Consequently, this switch set this point before the match start. 

* Strategy Configuration
It makes possible to choose between two kinds of implemented strategies before the match. For example, just before the start, we have the
possibility of aggressive or defensive strategy, function of the opponent. 

* Initialisation Switch

This switch launches an initialisation sequence. After the power up of the robot and before its first move, we wanted to have the possibility to check 
all different robot's systems. This switch indicates with a DEL if all systems are operational. This is a kind of automatic check list before the match 
and the first move of the robot. 

All these switches are linked to two DEL. First one to electrically confirm switches' position and the associated setting and the second one used to 
have the response of the robot operating system. With these two LEDs, we are sure that our robot understood all these parameters.

### Useful information
* Correspondence

Strategy  

Pin 1 - UP   = GREEN = 0 (out raspberry) - DOWN  = RED = 1 (out raspberry)

Initialisation

Pin 2 - UP   = GREEN = 0 (out raspberry) - DOWN  = RED = 1 (out raspberry)

Team

Pin 3 - LEFT = GREEN = 1 (out raspberry) - RIGHT = RED = 1 (out raspberry)

Start 

Pin 4 - piece IN     = 0 (out raspberry) - piece OUT   = 1 (out raspberry)

* Switch placement

The emplacement is taken when the splayed side is on the right then : 

1. Strategy switch is in the upper left corner 
2. Initialisation switch is on the right side of the strategy switch 
3. Team switch is in the lower left corner 

Finally, the starter switch is plugged on the board. It consists of a relay, sending a binary information to the ROS operating system. Linked to a 0.5m
cable, this system allows to launch the match sequence pulling on it. 

### Getting Started

The following code is launched with ROS (if you haven't read yet this section, it might be tricky) this is a simplified version of the code used in 
2018 robots. The aim of this code is the setting of a parameter in a configuration file in the first place, next the code publish on the start topic 
when the start switch is pulled. This last action indicates to all script listening this topic the beginning of a game. 

'''python 
import sys
import random
import rospy
from std_msgs.msg import Empty

raspberry = "-r" in sys.argv

pi = None
#If passed the -r argument, load the rapsberry libs
if raspberry:
    import pigpio
    pi = pigpio.pi()

rospy.init_node('startup_conf')

#Robot
robot = rospy.get_param("/robot")

pub_start = rospy.Publisher('start', Empty, queue_size=1)
pub_reset = rospy.Publisher('reset', Empty, queue_size=1)


#GPIO PIN CONFIGURATIONS
#This pin will be used to configure the team
pin_team = 23

#This pin will be used to drive a led to indicate that the team has been set correctly in ROS
pin_team_feedback = 21

#This pin will be used to drive a led to indicate that the start is understood by ROS
pin_strategy_feedback = 20

#This pin will be used to launch robot
pin_start = 12

#Function to set team parameter in the configuration file 
def update_team(gpio, level, tick):
    if raspberry:
        if level:
            rospy.set_param("/team", "red")
            rospy.set_param("/reset/position", rospy.get_param("/start/{}/red/position".format(robot)))
            pi.write(pin_team_feedback, 1)
        else :
            rospy.set_param("/team", "green")
            rospy.set_param("/reset/position", rospy.get_param("/start/{}/green/position".format(robot)))
            
   #Blink 2 times for acknowledge
            pi.write(pin_team_feedback, 0)
    else:
        choice = random.choice(["green", "red"])
        team = rospy.set_param("/team", choice)
        rospy.logwarn("Setting team automatically to: " + choice)
        if team == "green":
            rospy.set_param("/reset/position", rospy.get_param("/start/{}/green/position".format(robot)))
        elif team == "red":
            rospy.set_param("/reset/position", rospy.get_param("/start/{}/red/position".format(robot)))

    reset()


publish = True

#If high level on start pin this function publish on the start topic
def start(gpio, level, tick):
    global publish
    rospy.sleep(0.2)
    if raspberry and not pi.read(pin_start):
        if publish :
            pi.write(pin_strategy_feedback, 1)
            pub_start.publish(Empty())
            publish = False

#Function to reset all set parameters
def reset():
    pub_reset.publish(Empty())
    if raspberry:
        if pi.read(pin_start):
            global publish
            publish = True
            pi.write(pin_strategy_feedback, 0)

com = False
init = True

if raspberry and init:
    pi.set_mode(pin_team, pigpio.INPUT)
    pi.set_mode(pin_start, pigpio.INPUT)
    pi.set_mode(pin_team_feedback, pigpio.OUTPUT)
    pi.set_mode(pin_strategy_feedback, pigpio.OUTPUT)
    pi.set_pull_up_down(pin_start, pigpio.PUD_DOWN)
    pi.set_pull_up_down(pin_strategy_feedback, pigpio.PUD_DOWN)
    teamInterrupt = pi.callback(pin_team, pigpio.EITHER_EDGE, update_team)
    startInterrupt = pi.callback(pin_start, pigpio.EITHER_EDGE, start)

    update_team(None, pi.read(pin_team), None)
else:
    rospy.sleep(1)
    update_team(None, None, None)

while not rospy.is_shutdown():
    pass
'''

Below, you have the circuit to implement  between the board and the raspberry running ROS

[Start PCB Front](pictures/Start_F.JPG =100x20)

* The two couples of black and red wires are the 5V 
* The white wire is connected to the pin 12 on the raspberry and send startup interrupt information
* The red point is the pin normally connected to the 23rd pin on the raspberry and send team switch information
* The brown and green wires are connected to the pins 21 and 20 on the raspberry and connect acknowledge pins


If you copied the code and connect all cable as the pictures shows, the pulling of the start cable induce a publication on the start ROS topic (visible 
with an echo on this topic) and the modification of the switch place sets the team parameter ROS file. 

The switch strategy and initialisation were planned to respectively set a different implemented strategy and launch a initialisation (tests) 
sequence. It was not used in 2018 due to a lack of time but they are already on the PCB for a future robot.

Finally, in the real implementation on ROS, the pin configuration is placed in a .yaml file gathering all information required about raspberry pinout.
Then, the code used to get pin number is 

'''yaml
rospy.get_param("/path/name")
'''


[Github link for code](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/differential_driver/ros_packages/strategy/src/startup_conf.py)
--- [OneDrive link for Altium project](https://) ---

## Motor Board

The goal of this board is to bring individual intelligence for every motor used on our robots. It allows to associate a microcontroller (ATmega328)
with each motor. This main component is used for two aspects : regulation and communication. 

### Regulation

For the smaller robot, the operating system send speed consign to the four motors. Consequently, the goal for the motor is reaching this speed in a 
minimum of time and after conserving this value. The regulator implemented in the microcontrollers is Proportional Integral Derivative regulators. 
It's parameters allow to influence the way to consign reaching for the motor. Thanks to it, each motor, individually can follow the ROS order with a 
maximum of precision. 

PID's parameters were found with experimentation and all sources used for implementation are in the code file. 

### Communication

As said in the previous part, this is the operating system ROS the main brain of the robot. Consequently, this brain has to communicate with its slave.
The communication protocol used on our robots is SPI. It induces that a bus has to be built between all motor and ROS. Effectively, a broker gathers 
the signal for the motor but in any case each motor has to be linked to the bus. A role of the board is this connection via a SPI connector.

### Getting Started
To simplify the tutorial, we will use an arduino UNO AU LIEU DU ATmega328 microcontroller. The reason why we use a ATmega328 alone is the size of the 
system. The microcontroller and its oscillator are sufficient for our application and take less place on the robot. However, in this tutorial, we will
use the arduino UNO to slightly simplify manipulation.

Regarding the regulation, the following code implements the PID regulator. Its parameters were found after experimentation. It depends of the motor and the 
load. We simply adjusted the parameters until the motor behaviour was the expected one. (mr. Marchand knows the methodology ;) )

The blocks of the system are described below :

* Encoder

The encoder sends the rotation information to the controller. In our code, we used the _Encoder.h_ library to handle it. At this moment, we have the 
velocity of the wheel in rad/s.

* Motor control

The H bridge is used for power two motors at the same. One the left side of the board there are all control pins (PWM and DIR for the two motors)
and power supply of command (5V). On the other side of the board, you have the motor output and the motor power supply (24V).

'''cpp
#includes <Wire.h>
#includes <PID_v1.h>
#includes <ecamlib.h>

#include <Encoder.h>
#include <FlexiTimer2.h>

// Defines the pins to control the motor driver
// pin 5 controls the voltage to the motor by a PWM
// pin 4 controls the direction of the motor
const int PWM = 5;
const int DIR = 4;

// Define pin and tick per revolution of the encoder
Encoder motor_encoder = Encoder(2, 3);
const int ENCODER_TICKS_PER_REV = 3200;

// Defines the time between two samples in milliseconds for speed capture
const int CADENCE_MS = 50;
volatile double dt = CADENCE_MS / 1000.;

// Motor control variables and PID configuration
float motor_speed = 6.28319;
float gain_p = 15.0;
float gain_i = 5.0;
float gain_d = 0.5;

// Define angular velocity
volatile double omega;

// PID declaration function of used library
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,gain_p,gain_i,gain_d, DIRECT);

// Variable to keep track of the old encoder value
volatile long old_encoder = 0;

void setup() {
    // set timer 0 divisor to 1 for PWM frequency of 62500.00 Hz
    TCCR0B = TCCR0B & B11111000 | B00000001;

    pinMode(PWM,OUTPUT); 
    pinMode(DIR,OUTPUT);
 
 // Periodic execution of isrt() function thanks to FlexiTimer2 library    
    FlexiTimer2::set(CADENCE_MS, 1/1000., isrt); 
    FlexiTimer2::start();

    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(CADENCE_MS);

    // Motor direction 
    digitalWrite(DIR,LOW);
        
}

void loop() {
 
 Setpoint = (double) (abs(motor_speed));
    if (motor_speed > 0) {
        digitalWrite(DIR, LOW);
    } else {
        digitalWrite(DIR, HIGH);
    }
    
    Input = abs(omega);
    
    myPID.Compute();
    
    analogWrite(PWM, Output);
    
    Serial.print("Omega : ");
    Serial.println(omega);
    delay(CADENCE_MS);
}

// Speed measurement 
void isrt(){
    int deltaEncoder = motor_encoder.read() - old_encoder;
    old_encoder = motor_encoder.read();
    
    // Angular velocity 
    omega = ( (2.0 * 3.141592 * (double)deltaEncoder) / ENCODER_TICKS_PER_REV ) / dt;  // rad/s
}
''' 

Remark : All used library is on Github (see link at the end)


* Controller

Motor are controlled by the arduino with PWM signals. The implemented PID regulator computes the necessary duty cycle to send for reaching the target 
velocity function of its parameters. 


The following complete circuit is shown below

--- PIC OF THE CIRCUIT --- UNO + H Bridge + MOTOR - DRAW CONNECTION --- After robot's deconstruction

The implemented PID regulator computes to reach the target velocity   
So at this moment, you have an arduino UNO controlling motor in velocity thanks to a PID regulation. All regulation is visible in the Serial Window of
the Arduino software. 

Now the next step is to give the control of the motor velocity to ROS core on the raspberry. For that, we implemented a SPI connection with ROS. These 
concepts are explained in the ROS serial section. On the motor side, a function is added on the main code on the arduino UNO to receive data from master
and set value of PID (proportional gain, integrative gain, derivative gain and velocity setpoints.

'''cpp
// function that executes whenever data is received from the master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
    if (Wire.available() >= 2) {
        unsigned char reg = Wire.read();

        switch (reg) {
            // Configuration
            case 0x00:
                Serial.println("Tried to change the configuration, but it's not handled yet...");
                break;

            // Set motor speed
            case 0x10:
                // Expecting a 4 byte float value representing
                // angular velocity
                if (Wire.available() >= 4) {
                    unsigned char b0 = Wire.read();
                    unsigned char b1 = Wire.read();
                    unsigned char b2 = Wire.read();
                    unsigned char b3 = Wire.read();
                    motor_speed = bytesToFloat(b0, b1, b2, b3);
                    Serial.print("Motor speed set to ");
                    Serial.print(motor_speed, 3);
                    Serial.println(" rad/s");
                }
                break;

            // Set PID gains
            case 0x20:
                // Expecting a 4 byte float value representing
                // proportional gain for the PID
                if (Wire.available() >= 4) {
                    unsigned char b0 = Wire.read();
                    unsigned char b1 = Wire.read();
                    unsigned char b2 = Wire.read();
                    unsigned char b3 = Wire.read();
                    gain_p = bytesToFloat(b0, b1, b2, b3);
                    Serial.print("Proportional gain set to ");
                    Serial.println(gain_p, 3);
                }
                break;
            case 0x21:
                // Expecting a 4 byte float value representing
                // integral gain for the PID
                if (Wire.available() >= 4) {
                    unsigned char b0 = Wire.read();
                    unsigned char b1 = Wire.read();
                    unsigned char b2 = Wire.read();
                    unsigned char b3 = Wire.read();
                    gain_i = bytesToFloat(b0, b1, b2, b3);
                    Serial.print("Integral gain set to ");
                    Serial.println(gain_i, 3);
                }
                break;
            case 0x22:
                // Expecting a 4 byte float value representing
                // derivative gain for the PID
                if (Wire.available() >= 4) {
                    unsigned char b0 = Wire.read();
                    unsigned char b1 = Wire.read();
                    unsigned char b2 = Wire.read();
                    unsigned char b3 = Wire.read();
                    gain_d = bytesToFloat(b0, b1, b2, b3);
                    Serial.print("Derivative gain set to ");
                    Serial.println(gain_d, 3);
                }
                break;
            
            default:
                Serial.println("Unexcpected register access over I2C");
                break;
        }
    }
}
'''

### Upload protocol for ATmega 328 
If you understood the tutorial with arduino UNO and want to use the ATmega328, you just have to use PCB with connection show below and implement the 
code exposed above on the ATmega328. This mocrocontroller is programmed with the ISP Programmer explains in the relative section.

--- PIC OF THE CIRCUIT --- Motor board + H Bridge + MOTOR - DRAW CONNECTION --- After robot's deconstruction

[Github link for code](https://github.com/Ecam-Eurobot/Eurobot-2018/tree/arduino-pid/arduino)
--- [OneDrive link for Altium project](https://) ---

## Arduino Communication Shield - SPI Broker
Motor command communication consists of a two-level interface. ROS communicate via ROS Serial (explained in the appropriate section) implemented on an  arduino UNO and this arduino handle the communication with the four individual motors. To help for connections, we designed an arduino shield gathering simply all SPI bus connections. The only role of the PCB is deleting redundancy in the SPI bus connections and consequently facilitating the SPI bus  execution. As the PCB is really simple, it implements a bus SPI connection, it requires no explanation. 
The utility of this PCB concerns SPI and Ros Serial, you will find more explanation about these subjects in the related section. 

[Broker Front View](pictures/Broker_F.JPG =100x20) [Broker Top View](pictures/Broker_T.JPG =100x20) 
--- [OneDrive link for Altium project](https://) --


## If we had to redo it 
If we had to design another robot with technologies described, there are some decision that we would change. 

* PCB Design

The ECAM installation for PCB printing is very useful for prototyping. It is not comparable with PCB that specialised company can create. So, our advice is the following, as soon as a board is functional and definitive, send it to have a proper and reliable board. Moreover, try to reach this step as soon as possible. 

* Connection 

We have underestimated the connection on the different board. Regarding the JST connector, they are solid but hard to disconnect and cable creation is  long without the right tool. About the 10 pins shrouded header, they are not reliable enough. Opposite to the JST, disconnection is too easy. We really  advise against its use. They were the source of a lot of problems.
About the SPI connector (10 pins shrouded header) it was not strong enough. The contact failure brought a lot of problems in the communication between ROS  serial and ATmega.
In the two situations, we advise against the use of these two types of connector, there are a lot of alternative solutions. 

![JST connector](pictures/JST_Connector.JPG =100x20) 
![10 pins shrouded headers](pictures/10Pins_shroudedHeaders.JPG =100x20)
