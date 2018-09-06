#include <DynamixelMotor.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define RX 9
#define TX 10
#define CP 5
#define INT A5
#define RST 12


// --setting for sensor--
const float pitch_offset = 160.0;                                     //Dynamixel unit 1 = 0,088°
const float roll_offset = 0.0;
const float angle = 180.0;                                            //unit = degree
const float unit = 11.377778;

unsigned long oldT = 0;
unsigned long frequency = 1;                                         //How often the sensor will be checked (unit = Hz)


// --settings for Dynamixel--
const uint8_t id_pitch = 5;
const uint8_t id_roll = 1;
const uint32_t baudrate = 57142;

uint16_t approachSpeed = 150;
const static uint16_t levelSpeed = 1023;                              //1024 = MaxSpeed

const uint16_t homePositionPitch = 2048;
const uint16_t minValuePitch = 1400;
const uint16_t maxValuePitch = 2280;

const uint16_t homePositionRoll = 2048;
const uint16_t minValueRoll = 1448;
const uint16_t maxValueRoll = 2648;

const uint16_t restraint = 57;

uint16_t pitch = 0, roll = 0, oldPitch = 0, oldRoll = 0;



Adafruit_BNO055 bno = Adafruit_BNO055();                              //set ... of sensor

SoftwareDynamixelInterface interface(RX,TX,CP);                       //set (RX,TX,Controlpin)   ---hieß anders mit der alten ardyno Version -DynamixelInterface &interface=*createSoftSerialInterface(RX,TX,CP);

DynamixelMotor motor_pitch(interface, id_pitch);                      //set id of Dynamixel_pitch

DynamixelMotor motor_roll(interface, id_roll);                        //set id of Dynamixel_roll


void setup() {

  // --initialise the sensor--

  pinMode(INT, INPUT);
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);

  bno.begin();
  bno.setMode(0x8);
  delay(100);

  // --initialise the servos--

  interface.begin(baudrate);                                          //set Baudrate of Dynamixel
  delay(100);
  
  motor_pitch.init();
  motor_pitch.enableTorque();

  motor_roll.init();
  motor_roll.enableTorque();

  motor_pitch.jointMode(minValuePitch, maxValuePitch);                //choose 'jointMode' to enable 'goalPosition'
  motor_pitch.speed(approachSpeed);                                   //adjustment of speed of the Dynamixel [0.114rpm]
  motor_pitch.goalPosition(homePositionPitch);                        //drive to home position
  
  motor_roll.jointMode(minValueRoll, maxValueRoll);
  motor_roll.speed(approachSpeed);
  motor_roll.goalPosition(homePositionRoll);

  motor_pitch.led(LOW);
  motor_roll.led(LOW);
  
  delay(1000);

  motor_pitch.speed(levelSpeed);                                      //speed for levelling the laserscanner
  motor_roll.speed(levelSpeed);
}

void loop() 
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  roll = (uint16_t)((-(float)(euler.z()) + angle) * unit + roll_offset);
  pitch = (uint16_t)(((float)(euler.y()) + angle) * unit + pitch_offset);

  if(pitch < minValuePitch + restraint && pitch > maxValuePitch - restraint && roll < minValueRoll + restraint && roll > maxValueRoll - restraint)      //prevent driving into the stop of Dynamixel
  {
    if(pitch < minValuePitch + restraint)
    {
      pitch = minValuePitch + restraint;
    }

    if(pitch > maxValuePitch - restraint)
    {
      pitch = maxValuePitch - restraint;
    }

    if(roll < minValueRoll + restraint)
    {
      roll = minValueRoll + restraint;
    }

    if(roll > maxValueRoll - restraint)
    {
      roll = maxValueRoll - restraint;
    }
  }
  
  motor_pitch.goalPosition(pitch);
  motor_roll.goalPosition(roll);

  if(millis() - oldT > (1000/frequency))
  {
    if(oldPitch == pitch && oldRoll == roll)
    {
      sensorStatus();
    }
    oldPitch = pitch;
    oldRoll = roll;
    oldT = millis();
  }
  
}


void sensorStatus(void)
{  
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  if(system_status == 1)
  {
    reset_IMU();
  }
}


void reset_IMU()
{
  digitalWrite(RST, LOW);
  motor_pitch.led(HIGH);
  motor_roll.led(HIGH);

  delay(10);
  digitalWrite(RST, HIGH);
  delay(10);

  bno.begin();
  bno.setMode(0x8);
  delay(100);
}

