#include <ArduinoHardware.h>

#include <DynamixelMotor.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define RX 9
#define TX 10
#define CP 5

const uint16_t baudrateSerial = 57600;

uint8_t situation = 1;

bool result_roll = 1, result_pitch = 1;                                            //Values that shows moving of Dynamixel
bool flag = 0;
uint16_t oldPitch = 0, oldRoll = 0, pitchMotor = 0, rollMotor = 0;
unsigned long time;
const uint16_t duration = 750;


// --setting for sensor--
const float angle = 180.0;                                            //unit = degree
const float unit = 11.377778;


// --settings for Dynamixel--
const uint8_t id_pitch = 2;
const uint8_t id_roll = 1;
const uint32_t baudrate = 117647;

const uint16_t approachSpeed = 150;
const static uint16_t levelSpeed = 200;                              //1023 = MaxSpeed

const uint16_t homePosition = 2048;
const uint16_t minValue = 0;
const uint16_t maxValue = 4095;

const uint16_t movement = 340;

const uint8_t serviceT = 10;

uint16_t pitch = 0, roll = 0;


Adafruit_BNO055 bno = Adafruit_BNO055();                              //set ... of sensor

SoftwareDynamixelInterface interface(RX,TX,CP);                       //set (RX,TX,Controlpin)   ---hieß anders mit der alten ardyno Version -DynamixelInterface &interface=*createSoftSerialInterface(RX,TX,CP);

DynamixelMotor motor_pitch(interface, id_pitch);                      //set id of Dynamixel_pitch

DynamixelMotor motor_roll(interface, id_roll);                        //set id of Dynamixel_roll


void setup() 
{
  Serial.begin(baudrateSerial);
  
  // --initialise the sensor--

  bno.begin();
  bno.setMode(0x8);
  bno.setAxisRemap(0x21);
  bno.setAxisSign(0x02);
  delay(100);


  // --initialise the servos--

  interface.begin(baudrate);                                          //set Baudrate of Dynamixel
  delay(100);
  
  motor_pitch.init();
  motor_pitch.enableTorque();

  delay(100);
  
  motor_roll.init();
  motor_roll.enableTorque();

  motor_pitch.led(HIGH);
  motor_roll.led(HIGH);

  motor_pitch.jointMode(minValue, maxValue);                          //choose 'jointMode' to enable 'goalPosition'
  motor_pitch.speed(approachSpeed);                                   //adjustment of speed of the Dynamixel [0.114rpm]
  motor_pitch.goalPosition(homePosition);                        //drive to home position
  
  motor_roll.jointMode(minValue, maxValue);
  motor_roll.speed(approachSpeed);
  motor_roll.goalPosition(homePosition);
  
  motor_pitch.speed(levelSpeed);                                      //speed for levelling the laserscanner
  motor_roll.speed(levelSpeed);

  delay(1000);

  motor_pitch.led(LOW);
  motor_roll.led(LOW);
}

void loop() 
{ 
  Serial.print("Case: ");
  Serial.println(situation);
  
  Serial.println("--Motor--");
  Serial.print("Pitch: ");
  pitchMotor = motor_pitch.currentPosition();
  Serial.println(pitchMotor);
  
  /*Serial.print("    Roll: ");
  rollMotor = motor_roll.currentPosition();
  Serial.println(rollMotor);*/
  Serial.println("b");
  
  displaySensorEuler();

  displaySensorStatus();
  
  Serial.println("------------------------------------------");

  if(pitch == oldPitch || roll == oldRoll 
     || pitchMotor > homePosition + movement + 10|| pitchMotor < homePosition - movement -10)
     //|| rollMotor  > homePosition + movement + 10|| rollMotor  < homePosition - movement -10)
  {
    warnings();
  }
  else
  {
    flag = 0;
  }

  stateMachine();
}



void stateMachine()
{
  switch (situation)
  {
    case 1:

      motor_pitch.goalPosition(homePosition + movement);
      motor_roll.goalPosition(homePosition + movement);

      Serial.println(situation);

      delay(serviceT);
      motor_pitch.read(0x2E, result_pitch);                            
      delay(serviceT);
      motor_roll.read(0x2E, result_roll);

      if(!result_pitch && !result_roll)
      {
        situation = 2;
      }
    break;

    case 2:

      motor_pitch.goalPosition(homePosition - movement);
      motor_roll.goalPosition(homePosition - movement);
      
      Serial.println(situation);

      delay(serviceT);
      motor_pitch.read(0x2E, result_pitch);                            
      delay(serviceT);
      motor_roll.read(0x2E, result_roll);

      if(!result_pitch && !result_roll)
      {
        situation = 1;
      }
    break;

    default:
    break;
  }
}



void warnings()
{
  if(pitch == oldPitch || roll == oldRoll)
  {
    if(!flag)
    {
      time = millis();
      flag = 1;
    }
    
    if(millis() - time > duration)
    {
      Serial.println("                                                      --- WARNING - IMU ---");
      Serial.print("Zeit: ");
      Serial.println(millis()-time);
      motor_pitch.led(HIGH);
      motor_roll.led(HIGH);
    }
  }


  if(pitchMotor > homePosition + movement || pitchMotor < homePosition - movement)
  //|| rollMotor  > homePosition + movement || rollMotor  < homePosition - movement)
  {
    Serial.println("                                                      --- WARNING - MOTOR ---");
    
//    if(pitchMotor > homePosition + movement || pitchMotor < homePosition - movement)
//    {
      Serial.print("PitchMotor: ");
      Serial.println(pitchMotor);
/*    }
    else
    {
      Serial.print("RollMotor: ");
      Serial.println(rollMotor);
    }*/
  }
}


void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
}


void displaySensorEuler()
{

  oldPitch = pitch;
  oldRoll = roll;
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  pitch = euler.z();
  roll = euler.y();

  Serial.println("--IMU--");
  Serial.print("Z: ");
  Serial.print(pitch);  

  Serial.print("    Y: ");
  Serial.println(roll);
}

