/* Control of robtics arm using servos and joysticks 
 *
 *  by Godfrey Inyama
 *  for Group 13
 *
 *  A ring buffers are used to average the
 *  ADC readings from five potentiometers.
 *  This average is used to control
 *  speed of five hobby servos.
 *
 */

#include <Servo.h>

#define SERVO_X_PIN 12   //base                        
#define SERVO_Y_PIN 11   //elbow1 moving up from base                       
#define SERVO_Z_PIN 10   //elbow2 moving up from elbow1
#define SERVO_A_PIN 9    //elbow3 moving up from elbow2
Servo SERVO_B_PIN;       //end effector moving up from elbow3

// Joysticks assigned to analog pins
// with JOYSTICK_B_PIN assigned for digital read

const byte JOYSTICK_X_PIN = A5;                       
const byte JOYSTICK_Y_PIN = A4;                       
const byte JOYSTICK_Z_PIN = A1;
const byte JOYSTICK_A_PIN = A0;
const byte JOYSTICK_B_PIN = 3;

// set the direction for individual servo
const byte SERVOS_IN_USE = 4;
const boolean SERVO_REVERSE_FLAG[] = {1, 0, 0, 0, 0}; //set to one if servo should be reversed

const long MIN_PULSE[] = {850, 850, 850, 850};
const long MAX_PULSE[] = {2100, 2100, 2100, 2100};                     
const long MIN_POT = 0;                        
const long MAX_POT = 1023;                       
const long POWER_OF_TWO_TO_AVERAGE = 4;          

boolean state = false;  //initialize the state if the push button

// Changing "POWER_OF_TWO_TO_AVERAGE" changes several other constants.
// The constants "BUFFER_SIZE" and "BUFFER_LIMIT" are calculated based on "POWER_OF_TWO_TO_AVERAGE".
//long SERVO_PULSE_RANGE[SERVOS_IN_USE]; 
// This isn't really a consstant but we only set the value once.


const byte SERVO_PIN[] = {SERVO_X_PIN, SERVO_Y_PIN, SERVO_Z_PIN, SERVO_A_PIN};
const byte JOYSTICK_PIN[] = {JOYSTICK_X_PIN, JOYSTICK_Y_PIN, JOYSTICK_Z_PIN, JOYSTICK_A_PIN};

//const long POT_RANGE = MAX_POT - MIN_POT;

const int BUFFER_SIZE = 1 << POWER_OF_TWO_TO_AVERAGE; // Do not change.
const int BUFFER_LIMIT = BUFFER_SIZE - 1;             // Do not change.

// Time constants and variables should be unsigned longs.
const unsigned long ANALOG_READ_PERIOD = 5000;  // read pots at 200Hz "ANALOG_READ_PERIOD" must be <= "DEBUG_PERIOD"
const unsigned long DEBUG_PERIOD = 100000;      // update serial at 4Hz "DEBUG_PERIOD" must be >= "SERVO_PERIOD"
const unsigned long SERVO_PERIOD = 20000;       // update servo at 50Hz

const long LOW_CENTER_THRESHOLD = 477;     
const long HIGH_CENTER_THRESHOLD = 515;    
const long POT_TO_SPEED_CONSTANT = 20;           // regulate servo speed, higher value = slower speed.

long averagingBuffer[SERVOS_IN_USE][BUFFER_SIZE];
int bufferIndex = 0;
long servoPosition[SERVOS_IN_USE];

long bufferTotal[SERVOS_IN_USE];

unsigned long lastDebug;
unsigned long lastServo;
unsigned long lastAnalogRead;

Servo myServo[SERVOS_IN_USE];

void setup()
{
  SERVO_B_PIN.attach (8); //for digital read for end effector
  pinMode (JOYSTICK_B_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  for (int i = 0; i < SERVOS_IN_USE; i++)
  {
    servoPosition[i] = MIN_PULSE[i] + ((MAX_PULSE[i] - MIN_PULSE[i]) / 2);
    myServo[i].writeMicroseconds(servoPosition[i]); // start servo in center position
    myServo[i].attach(SERVO_PIN[i], MIN_PULSE[i], MAX_PULSE[i]);
    bufferTotal[i] = 0;
    for (int j; j < BUFFER_SIZE; j++) // Fill buffer with start position.
    {
      averagingBuffer[i][j] = (MAX_POT - MIN_POT) / 2;
      bufferTotal[i] += averagingBuffer[i][j];
    }
  }

  lastDebug = micros();
  lastServo = lastDebug;
  lastAnalogRead = lastDebug;
}

void loop()
{
  checkAnalogReadTime();  //calling analog read function below
  checkDigitalReadTime(); //calling digital read function below
}

void checkAnalogReadTime()
{
  if (micros() - lastAnalogRead > ANALOG_READ_PERIOD)
  {
    lastAnalogRead += ANALOG_READ_PERIOD;
    long joystickInput;

    bufferIndex++;
    bufferIndex &= BUFFER_LIMIT;

    for (int i = 0; i < SERVOS_IN_USE; i++)
    {
      joystickInput = analogRead(JOYSTICK_PIN[i]);
      if (SERVO_REVERSE_FLAG[i])
      {
        joystickInput = MAX_POT - joystickInput;
      }
      bufferTotal[i] -= averagingBuffer[i][bufferIndex]; // out with the old
      averagingBuffer[i][bufferIndex] = joystickInput;
      bufferTotal[i] += averagingBuffer[i][bufferIndex]; // in with the new
    }
    checkServoTime();
  }
}

void checkDigitalReadTime()
{
  if (digitalRead(JOYSTICK_B_PIN) == LOW) {

  if (state == false) {
    state = true;
    SERVO_B_PIN.write (70);
    delay (500);
  } else {
    state = false;
    SERVO_B_PIN.write(22);
    delay(500);
  }
 }
}


void checkServoTime()
// Called from "checkAnalogReadTime" function.
{
  if (micros() - lastServo > SERVO_PERIOD)
  {
    lastServo += SERVO_PERIOD;
    controlServo();
  }
}


void controlServo()
// Called from "checkServoTime" function.
{
  long average;
  long servoSpeed;
  boolean debugFlag = checkDebugTime();
  
  for (int i = 0; i < SERVOS_IN_USE; i++)
  {
    average = bufferTotal[i] >> POWER_OF_TWO_TO_AVERAGE;
    if (average < LOW_CENTER_THRESHOLD)
    {
      servoSpeed = (average - LOW_CENTER_THRESHOLD) / POT_TO_SPEED_CONSTANT;
      // negative speed proportional to distance from center pot
    }
    else if  (average > HIGH_CENTER_THRESHOLD)
    {
      servoSpeed = (average - HIGH_CENTER_THRESHOLD) / POT_TO_SPEED_CONSTANT;
      // positive speed
    }
    else // pot in dead zone
    {
      servoSpeed = 0;
    }
    servoPosition[i] += servoSpeed;
    if (servoPosition[i] > MAX_PULSE[i])
    {
      servoPosition[i] = MAX_PULSE[i];
    }
    else if (servoPosition[i] < MIN_PULSE[i])
    {
      servoPosition[i] = MIN_PULSE[i];
    }
    myServo[i].writeMicroseconds(servoPosition[i]);
    if (debugFlag)
    {
      //debugServo(i, average, servoPosition[i], servoSpeed);
    }
  }

}

boolean checkDebugTime()
// Called from "controlServo" function.
// This method checks to see if it's time to
// display data.
{
  boolean debugFlag = 0;
  if (micros() - lastDebug > DEBUG_PERIOD)
  {
    lastDebug += DEBUG_PERIOD;
    debugFlag = 1;
  }
  return debugFlag;
}
 

void debugServo(int servoIndex, long average, long servoOutput, long servoSpeed)
// Called from "controlServo" function.
// Serial output slows down code execution.
// It would probably be a good idea to remove this section of code
// once the program is working as hoped and when serial
// output is now longer desired.
{
    Serial.print(F("servo # "));
    Serial.print(servoIndex, DEC);
    Serial.print(F(": average = "));
    Serial.print(average, DEC);
    Serial.print(F(", position = "));
    Serial.print(servoOutput, DEC);
    Serial.print(F(", spead = "));
    Serial.println(servoSpeed, DEC);
}
