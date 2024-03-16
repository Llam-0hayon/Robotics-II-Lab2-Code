/*
Car Follow For Elegoo Smart Car V4.0 

The purpose of this code is to make a eleggo smart car V4.0 follow a onjext or person within the range of the cars ultra sonic sensor.

The Code mainly usese a series of if  statements to do the logic for deciding which direction to go in when to move in tht a direction and when to stop.
The movement id done mostly by the creation and calling of functions for each direction that car moves in.
ASweel as a function for finding the shortest distnce.
In summary This mostly requires:
-if else
-Servo.h Library
-digitaWrite 
-analogWrite

The main loop of the code functions by having the servo on whcih the ultra sonic sensor is attached to move forward the left then right.
A measuremt is taken and saved at eaxh direction the servo is pointed in.
Then a fucntion is called that finds the shortest distnace out of the three then moves then rotates t he car in the shortest direction. 
The car will be allighned to whwre the ultrasonic snesor was when it had taken that measurement


The pins for the ultra sonic sensor:
- Vcc pin  =  pin 1 = 5V+
- Trig pin =  pin 2 = pin 13 on arduino
- Echo pin =  pin 3 = pin 12 on arduino
- GND pin  =  pin 4 = GND

Pins for servo motor(P10)
- GND pin    = pin 1 = GND
- Vcc pin    = pin 2 = 5V+
- Signal pin = pin 3 = pin 10 on a arduino

Pins for dc motors
- AIN1(Direction)             = pin 10 on TB6612FNG = pin 8 on arduino
- PWMA(Speed/amount of power) = pin 9 on TB6612FNG  = pin 5 on arduino
- BIN1(Direction)             = pin 6 on TB6612FNG  = pin 7 on arduino
- PWMB(Speed/amount of power) = pin 7 on TB6612FNG  = pin 6 on arduino
- STBY                        = pin 19 on TB6612FNG = pin 3 on arduino

Go to: https://www.elegoo.com/blogs/arduino-projects/elegoo-smart-robot-car-kit-v4-0-tutorial
For more info on components pin configuration/Data sheets aswell as info on the follow funtion and other functions this car 
can perform.
*/

// Including the Servo library for controlling the servo motor.
#include <Servo.h>
// Creating a Servo object named myservo to control the servo motor.
Servo myservo; 

// Defining pin numbers for motor control and standby.
int PWMA = 5;                                               // speed/power for motor A.
int AIN1 = 7;                                               // Direction motor B.
int BIN1 = 8;                                               // Direction motor A.
int PWMB = 6;                                               // Speed?power for motor b.
int STBY = 3;                                               // Stand by pin (allows power to motors).

// Defining speed values for different motor speeds.
int MINspeed = 50;                                          // minimum speed before motors struggle.
int SPEED1 = 100;                                           // A speed value.
int SPEED2 = 150;                                           // A speed value.
int SPEED3 = 200;                                           // A speed value.
int MAXspeed = 255;                                         // Define the maximum motor speed.

// Define amount of milliseconds to go 180 degrees when going right
int oneEighty = 300; 

// Define the maximum distance for obstacle detection.
int maxDis = 15;
int minDis = 3;

// Defining pin numbers for ultrasonic sensor trigger, echo, and servo signal.
#define trigPin 13                                                                  //Trigger pin
#define echoPin 12                                                                  //Echo pin
#define servoSig  10                                                                // Signal pin 

// Below create a function to make the car stop
void stop() {
  
  digitalWrite(STBY, 1);   // Give power to standby pin

  // motor A
  pinMode(PWMA, OUTPUT);   // Set PWMA pin as OUTPUT
  analogWrite(PWMA, 0);    // Set the speed of motor A to zero
  pinMode(AIN1, OUTPUT);   // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 1);   // Set the direction of motor A (Set it HIGH to rotate clockwise)

  // Motor B
  pinMode(PWMB, OUTPUT);   // Set PWMB pin as OUTPUT
  analogWrite(PWMB, 0);    // Set the speed of motor B to zero
  pinMode(BIN1, OUTPUT);   // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 1);   // Set the direction of motor B (Set it HIGH to rotate clockwise)
}

// Create a function to make the car move Forward with a variable for speed whos value can be defined when called.
void forward(int speed) {
  
  digitalWrite(STBY, 1);       // Give power to standby pin

  // motor A
  pinMode(PWMA, OUTPUT);       // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);    // Set the speed of motor A using PWM
  pinMode(AIN1, OUTPUT);       // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 1);       // Set the direction of motor A (Set it HIGH to rotate clockwise)

  // Motor B
  pinMode(PWMB, OUTPUT);       // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);    // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);       // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 1);       // Set the direction of motor B (Set it HIGH to rotate clockwise)
}

//Create a function to make the car move Left with a variable for speed whos value can be defined when called.
//moving left means spinnig in place clockwise.
//This is done by the left side motors(B) move counter clockwise and having the right side motors(A) move clockwise.
  void left(int speed) {
  
  digitalWrite(STBY, 1);     // Give power to standby pin   

  // motor A
  pinMode(PWMA, OUTPUT);     // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);  // Set the speed of motor A using PWM
  pinMode(AIN1, OUTPUT);     // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 1);     // Set the direction of motor A (Set it HIGH to rotate clockwise)    
    

  // Motor B
  pinMode(PWMB, OUTPUT);     // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);  // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);     // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 0);     // Set the direction of motor B (Set it LOW to rotate counter clockwise)
}

//Create a function to make the car move Right with a variable for speed whos value can be defined when called.
//moving left means spinnig in place counter clockwise.
//This is done by the left side motors(B) move clockwise and having the right side motors(A) move counter clockwise.
void right(int speed) {
  
  digitalWrite(STBY, 1);     // Give power to standby pin   

  // motor A
  pinMode(PWMA, OUTPUT);      // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);   // Set the speed of motor A using PWM
  pinMode(AIN1, OUTPUT);      // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 0);      // Set the direction of motor A (Set it LOW to rotate clockwise)    
    

  // Motor B
  pinMode(PWMB, OUTPUT);        // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);     // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);        // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 1);        // Set the direction of motor B (Set it HIGH to rotate counter clockwise)
}

// Create a function to make the car move Backward with a variable for speed whos value can be defined when called.
  void backward(int speed) {
  
  digitalWrite(STBY, 1);       // Give power to standby pin

  // motor A
  pinMode(PWMA, OUTPUT);       // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);    // Set the speed of motor A using PWM
  pinMode(AIN1, OUTPUT);       // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 0);       // Set the direction of motor A (Set it LOW to rotate counter clockwise)

  // Motor B
  pinMode(PWMB, OUTPUT);       // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);    // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);       // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 0);       // Set the direction of motor B (Set it LOW to rotate counter clockwise)
}

// Create a function to make the servo move to the middle.
void servoMiddle() 
{ 
  
  myservo.write(80);                                    //Set the servo to 90 degrees.
}                                  

// Create a function to make the servo move to the right.
void servoRight() {

  myservo.write(0);                                     //Set the servo to 0 degrees.
}
__
// Create a function to make the servo move to the left.
void servoLeft() {
  
  myservo.write(175);                                   //Set the servo to 180 degrees.
}

// create a function to measure the distance using the ultrasonic sensor.
float distanceRead() {
  digitalWrite(trigPin, LOW);                           // Set trigPin to LOW to ensure a clean pulse.
  delayMicroseconds(2);                                 // Short delay to allow any lingering signals to settle.
  digitalWrite(trigPin, HIGH);                          // Generate a 10-microsecond pulse by setting trigPin to HIGH.
  delayMicroseconds(10);                                // Generate a 10-microsecond pulse by setting trigPin to HIGH.
  digitalWrite(trigPin, LOW);                           // Reset trigPin to LOW to complete the pulse.
  long duration = pulseIn(echoPin, HIGH);               // Measure the duration of the pulse received on echoPin.
  float distance = (duration * 0.034 / 2);              // Calculate the distance using the speed of sound (0.0343 cm/microsecond.
  Serial.print("inside func = ");
  Serial.println(distance);
  return distance;                                      // Convert the distance to an integer and return the result.
} 

void setup() {
  myservo.attach(servoSig);                            // Attaches the servo motor to its signal pin
  Serial.begin(9600);                                  // Starts serial communication at 9600 baud rate
  pinMode(trigPin, OUTPUT);                            // Sets trigPin as an output
  pinMode(echoPin, INPUT);                             // Sets echoPin as an input
}

void loop() {
  servoMiddle();                                       // Moves the servo to the middle position
  delay(800);                                          // Delays for 800 milliseconds
  float dF = distanceRead();                           // Reads the distance in front of the sensor
  Serial.print("dF=");                                 // Prints the distance in front of the sensor
  Serial.println(dF);

  servoRight();                                        // Moves the servo to the right position
  delay(800);                                          // Delays for 800 milliseconds
  float dR = distanceRead();                           // Reads the distance to the right of the sensor
  Serial.print("dR=");                                 // Prints the distance to the right of the sensor
  Serial.println(dR);
 
  servoLeft();                                         // Moves the servo to the left position
  delay(800);                                          // Delays for 800 milliseconds
  float dL = distanceRead();                           // Reads the distance to the left of the sensor
  Serial.print("dL=");                                 // Prints the distance to the left of the sensor
  Serial.println(dL);

  shortDisFind(dL, dF, dR);                           // Finds the shortest distance among the three readings
  servoMiddle();                                      // Moves the servo to the middle position
  delay(800);                                         // Delays for 800 milliseconds
 
  float dx = distanceRead();                          // Reads the distance in the forward direction
  if (minDis <= dx && dx <= maxDis) {                 // Checks if the distance is within the acceptable range
    forward(80);                                      // Moves forward with a speed of 80
    Serial.print("FOR=");                             // Prints the forward distance
    Serial.println(dx);
  } else if (1 >= dx || minDis >= dx) {               // Stops and moves backward if the distance is too short
    stop();
    backward(80);
    delay(300);
  } else if (dx >= maxDis) {                          // Stops if the distance is too long
    stop();
  }
}









