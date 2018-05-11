# Ball Gun

![alt text](mechanical/2018/BallGun_SRC/BallGun3D.png )

## 3D sketch

all the parts of the ball gun is designed on Fusion 360 and printed on a 3D printer except of the balls and the wheel.

## Why a ball gun?

For the 2018 contest, we had to collect and sort balls by color and then put all the balls collected in the color of the team previously known in a big tank that made reference of a water castle as the balls made reference of water itself. The more balls we threw in the tank, the more points we earned.

We choose to throw the balls with a gun made of a ramp for the direction of the ball and a wheel with a DC motor. as shown on the picture, you can see a fixed ramp, an adjustable ramp and the wheel.

## Construction step by step

After lots of discussions with the team, I decided to choose the ball gun to complete the task of throwing balls in the tank. So I first made a prototype of the ball gun made of wood and a small DC motor I found in the lab but for the first tests it appeared that the small DC motor wasn't powerfull enough. So I choose a more powerfull and a better speed motor as a Maxon motor found in the lab and the results were very conclusive. 

![alt text](mechanical/2018/BallGun_SRC/BallGunPrototype.jpg )

The next step was to control the speed of the wheel so we could control the distance the ball make after throwing by the ball gun.

I used a small motor driver as the L293D which the specifications were in agreement with the needs of the DC motor. To control the H bridge driver, I used a simple arduino uno with a PWM signal so we could test different speeds of the motor. 

![alt text](mechanical/2018/BallGun_SRC/SchemaBlock.png )

The precision of the distance the ball make was made experimentaly when we did the different tests.

The code I first used to test the driver with the motor is the next code in arduino :

```
int inputPin = A0;  // set input pin for the potentiometer
int inputValue = 0; // potentiometer input variable

const int motorPin1  = 5;  // Pin 14 of L293
const int motorPin2  = 6;  // Pin 10 of L293

void setup() {
     // declare the ledPin as an OUTPUT:
     pinMode(motorPin1, OUTPUT);
     
}

void loop() {
     // read the value from the potentiometer:
     inputValue = analogRead(inputPin);

     // send the square wave signal to the LED:
     analogWrite(motorPin1, inputValue);

  digitalWrite(motorPin1, HIGH);
  delayMicroseconds(inputValue); // Approximately 10% duty cycle @ 1KHz
  digitalWrite(motorPin1, LOW);
  delayMicroseconds(1023 - inputValue);
     
}
```
When the tests were conclusive enough, we designed a ramp that fit in the robot correctly with an adjustable ramp for the ball to go higher or lower in case we need to modify the distance during the competition.

First we made the fixed ramp that comes from the ball sorting mecanism and end around the wheel.

![alt text](mechanical/2018/BallGun_SRC/FixedRamp.png )

After that we made a straight ramp that shows the ball the direction it must take.

![alt text](mechanical/2018/BallGun_SRC/StraightRamp.png )

And finaly, we designed a small piece that maintain the straight ramp from below and that you can adjust.

![alt text](mechanical/2018/BallGun_SRC/PieceAdjust.png )

After that, all we had to do is to install the gun in the robot and make some tests to determine the best power to give as a PWM to the motor to throw the balls from the distance chosen in the tank and to incorporate the code of the ball gun in the main code of the robot.
