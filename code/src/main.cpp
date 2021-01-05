/*  this code was created to be an interface between
*   the open source tool changing system for pick and place robots
*   that I developed during my bachelor's thesis.
*   it allows everyone to use a tool change system for their robots
*   that is completly 3D printed and rather customizable.

*   the program can check if a tool is mounted to the robot,
*   switch a tool after receiving a message from the robot
*   and confirm the tool change to the robot.
*   If the tool is lost during the robots program cycle, 
*   the toolchanger will send an emergency halt message to the robot.
*
*   developer: Jannes Kai Briese
*   version: 1.0
*   date of creation: December 24, 2020
*/

#include <Arduino.h>
#include <Servo.h> // servo library

Servo myServo; //   initialize servo lib

//  =====    variables  =====
const int lock = 57;   // servo angle for locked toolchanger      (tool mounted)
const int noLock = 15; // servo angle for unlocked toolchanger (no tool mounted)

const int thresh = 800;    //  threshhold for the proximity sensor
const int timeMax = 10000; //  timer for periodic check of tool status
int time = 0;              //  current time
bool status = 0;           //  saves the status of the toolchanger (0 = no tool mounted)

//  =====    pin declaration    =====
const int signalLED = LED_BUILTIN; // uses the LED on the arduino to visualize status

const int signalInPin = 2; // receives signals from the robot
const int sensorPin = A3;  // proximity sensor pin
const int servoPin = 5;    // servo pwm pin

const int relayK1 = 3; // signal to robot -> confirm
const int relayK2 = 4; // signal to robot -> emergency stop
const int relayK3 = 7; // activates power for a mounted tool

//  =====   functions   =====

/*  send function:
*   sends a signal to the robot
*   1 = confirm
*   0 = emergency stop
*/
void sendRoboSig(bool signal)
{
    //  if input variable is true, send confirm message
    if (signal)
    {
        digitalWrite(relayK1, HIGH);
        delay(200);
        digitalWrite(relayK1, LOW);
        Serial.print("sent message 'confirmed!' to robot");
    }

    //  else send signal to robot for an emergency stop
    else
    {
        digitalWrite(relayK2, HIGH);
        delay(200);
        digitalWrite(relayK2, LOW);
        Serial.print("sent message 'emergency stop!' to robot");
    }
}

/*  servo switch function
*   switches the position of the servo 
*   between locked and not locked position
*   according to input
*/
void changeServo(bool tool)
{
    int angle = 0;
    if (tool)
        angle = lock;
    else
        angle = noLock;

    myServo.write(angle);
    delay(500);
}

//  changes the status of the tool changer depending on the last status
void changeStatus()
{
    Serial.print("changed status from " + status);

    if (status)
    {
        status = 0;
    }

    else
    {
        status = 1;
    }

    changeServo(status); //  switches the servo to the new position
    sendRoboSig(true);   //  confirms change to robot
    Serial.println(" to " + status);
}

/*  read function:
*   reads the signals send from robot 
*   and starts a toolchange 
*   if a signal is received
*/
void readRoboSig()
{
    if (digitalRead(signalInPin))
    {
        changeStatus();
    }
}

/*  sensor function:
 *  reads the proximtity sensors signal and averages it over <checks> measurements
 *  to eliminate the possibility of a faulty signal
 *  then returns it
*/
int sensor()
{
    int checks = 10; // number of checks performed
    int temp = 0;    // temporary saves the value of the measurements

    // takes multiple measurements of the sensor value
    for (int i = 0; i < checks; i++)
    {
        Serial.print(analogRead(sensorPin) + " - "); //  sends the sensors value to the serial monitor
        temp += analogRead(sensorPin);               //  adds the new value to the old value
        delay(10);
    }

    temp /= checks;                //    average the value of the sensor over the number of checks performed
    Serial.println(" => " + temp); //    send value to serial
    return temp;                   //    returns the value
}

//  checks if a tool is mounted to the tool changer
bool checkTool()
{
    // if no tool is mounted, the value will be 4095;
    if (sensor() >= thresh)
    {
        digitalWrite(signalLED, HIGH);
        return false;
    }

    // any other value will cause the toolchanger to lock
    else
    {
        digitalWrite(signalLED, LOW);
        return true;
    }
}

/*  timer function
*   checks the time since last check 
*   and reads the state of the tool
*   sends emergency halt if not as exspected
*/
void checkTime()
{
    //  after timer runs out
    if (millis() - time >= timeMax)
    {
        //  checks if the tool is mounted like expected
        if (checkTool() != status)
        {
            sendRoboSig(0); //  sends a emergency halt to the robot
        }

        time = millis(); //  resets the timer
    }
}

//  =====   setup function  =====
//  runs once at the start of the microcontroller
void setup()
{
    // start serial for readout
    Serial.begin(9600);

    // declare pin modes
    pinMode(signalInPin, INPUT);
    pinMode(sensorPin, INPUT);
    pinMode(signalLED, OUTPUT);
    pinMode(servoPin, OUTPUT);
    pinMode(relayK1, OUTPUT);
    pinMode(relayK2, OUTPUT);
    pinMode(relayK3, OUTPUT);

    //  shut pins off
    digitalWrite(signalLED, LOW);
    digitalWrite(relayK1, LOW);
    digitalWrite(relayK2, LOW);
    digitalWrite(relayK3, LOW);

    // connect servo pin to servo
    myServo.attach(
        servoPin, 1000,
        2000); // map the pwm signal according to the datasheet of the servo

    // start of program
    status = checkTool(); //    checks if a tool is mounted and saves this information to 'status'
    changeServo(status);  //    turns the servo to the specified angle according to the state of the tool
}

//  =====   loop function   =====
//  repeated periodically
void loop()
{
    readRoboSig(); //  reads the signal from the robot and changes 'status' if its triggered

    checkTime(); //  checks the toolstatus periodically and sends emergency halt if needed

    delay(200); //  small delay for controller
}