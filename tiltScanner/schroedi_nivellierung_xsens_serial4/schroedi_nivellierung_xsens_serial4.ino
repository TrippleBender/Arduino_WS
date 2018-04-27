#include <DynamixelMotor.h>

#define RX 9
#define TX 10
#define CP 5

// --setting for Serial-communication--
const uint16_t baudrateSerial = 9600;
const uint8_t storage = 6;
char inputValue[storage];
const char check = '~';
uint8_t counter = 0;


// --setting for sensor--
const float pitch_offset = 0.0;                                       //Dynamixel unit 1 = 0,088°
const float roll_offset = 11.0;
const float angle = 180.0;                                            //unit = degree
const float unit = 11.377778;


// --settings for Dynamixel--
const uint8_t id_pitch = 2;
const uint8_t id_roll = 1;
const uint32_t baudrate = 117647;


uint16_t approachSpeed = 150;
const static uint16_t levelSpeed = 1023;                              //1023 = MaxSpeed

const uint16_t homePositionPitch = 2048;
const uint16_t minValuePitch = 1400;
const uint16_t maxValuePitch = 2280;

const uint16_t homePositionRoll = 2048;
const uint16_t minValueRoll = 1448;
const uint16_t maxValueRoll = 2648;

const uint16_t restraint = 57;                                        //restraint of 5° before stop collar of Dynamixel

float x = 0, y = 0, z = 0, w = 0;
float pitch = 0, roll = 0, yaw = 0;



SoftwareDynamixelInterface interface(RX,TX,CP);                       //set (RX,TX,Controlpin)   ---hieß anders mit der alten ardyno Version -DynamixelInterface &interface=*createSoftSerialInterface(RX,TX,CP);

DynamixelMotor motor_pitch(interface, id_pitch);                      //set id of Dynamixel_pitch

DynamixelMotor motor_roll(interface, id_roll);                        //set id of Dynamixel_roll


void setup() 
{
  Serial.begin(baudrateSerial);
  
  // --initialise the servos--

  /*interface.begin(baudrate);                                          //set Baudrate of Dynamixel
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

  delay(1000);
    
  motor_pitch.speed(levelSpeed);                                      //speed for levelling the laserscanner
  motor_roll.speed(levelSpeed);*/
}

float grab_fields()
{
  float tmp = 0.00;
  
  for(uint8_t i = 0; i < storage; i++)
    {
    inputValue[i] = Serial.read();

    if(inputValue[i] == check)
    {
      Serial.println("ERROR: missed data!");
      break;
    }
      
    tmp *= 10;
    
    if(inputValue[i] >= '0' && inputValue[i] <= '9')
    {
      tmp += inputValue[i] - '0';
    }
  }
    
  if(Serial.read() == check)
  {
    tmp /= (10 * storage);
    return tmp;
  }

  Serial.println("ERROR: missing end of message");
  return -10;
}


void loop() 
{  
  if(Serial.read() == check)
  {
    switch(Serial.read())
    {
      case 'w':
        w = grab_fields();
        ++counter;
        break;

      case 'x':
        x = grab_fields();
        ++counter;
        break;

      case 'y':
        y = grab_fields();
        ++counter;
        break;

      case 'z':
        z = grab_fields();
        ++counter;
        break;

      default:
        Serial.println("ERROR: missing clear assignment!");
        break;
    }
  }

  if(counter == 4)
  {
    counter = 0;
    
    // roll (x-axis rotation)
    float sinr = +2.0 * (w * x + y * z);
    float cosr = +1.0 - 2.0 * (x * x + y * y);
    roll = atan2(sinr, cosr) * (180/PI);
   
    // pitch (y-axis rotation)
    float sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
      pitch = copysign(M_PI / 2, sinp) * (180/PI);                      //use 90 degrees if out of range
    else
      pitch = asin(sinp) * (180/PI);

    // yaw (z-axis rotation)
    float siny = +2.0 * (w * z + x * y);
    float cosy = +1.0 - 2.0 * (y * y + z * z);  
    yaw = atan2(siny, cosy) * (180/PI);
  
    Serial.print("Roll = ");
    Serial.print(roll);
    Serial.print("    Pitch = ");
    Serial.print(pitch);

    Serial.print("    Yaw = ");
    Serial.println(yaw);


    pitch = (uint16_t)((-yaw + angle) * unit + pitch_offset);
    roll = (uint16_t)((-roll + angle) * unit + roll_offset);

    Serial.print("Roll = ");
    Serial.print(roll);

    Serial.print("    Pitch = ");
    Serial.println(pitch);
    Serial.println("---------------------------------------");
   
    /* if(pitch < minValuePitch + restraint && pitch > maxValuePitch - restraint && roll < minValueRoll + restraint && roll > maxValueRoll - restraint)      //prevent driving into the stop of Dynamixel
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
    motor_roll.goalPosition(roll);*/
  }
}


