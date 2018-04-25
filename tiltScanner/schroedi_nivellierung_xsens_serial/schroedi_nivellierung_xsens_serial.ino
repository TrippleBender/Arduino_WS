#define USE_USBCON

#include <ros.h>
#include <ArduinoHardware.h>

#include <DynamixelMotor.h>

#include <std_msgs/UInt16.h>
#include <sensor_msgs/Imu.h>


#define RX 9
#define TX 10
#define CP 5

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

double x = 0, y = 0, z = 0, w = 0;
double pitch = 0, roll = 0, yaw = 0;



ros::NodeHandle nh;

// --Subscriber--

void suborientation(const sensor_msgs::Imu sub_orientation)
{
  x = sub_orientation.orientation.x;
  y = sub_orientation.orientation.y;
  z = sub_orientation.orientation.z;
  w = sub_orientation.orientation.w;
}
ros::Subscriber<sensor_msgs::Imu> subOrientation("imu/data", &suborientation);


// --Publisher--

std_msgs::UInt16 pub_roll;
ros::Publisher pubRoll("roll", &pub_roll);

std_msgs::UInt16 pub_pitch;
ros::Publisher pubPitch("pitch", &pub_pitch);

std_msgs::UInt16 pub_yaw;
ros::Publisher pubYaw("yaw", &pub_yaw);


SoftwareDynamixelInterface interface(RX,TX,CP);                       //set (RX,TX,Controlpin)   ---hieß anders mit der alten ardyno Version -DynamixelInterface &interface=*createSoftSerialInterface(RX,TX,CP);

DynamixelMotor motor_pitch(interface, id_pitch);                      //set id of Dynamixel_pitch

DynamixelMotor motor_roll(interface, id_roll);                        //set id of Dynamixel_roll


void setup() {
  
  nh.initNode();

  // --Subscriber--
  
  nh.subscribe(subOrientation);

  // --Publisher--

  nh.advertise(pubRoll);
  nh.advertise(pubPitch);
  nh.advertise(pubYaw);


  // --initialise the servos--

  interface.begin(baudrate);                                          //set Baudrate of Dynamixel
  delay(100);
  
  /*motor_pitch.init();
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

void loop() 
{
  // roll (x-axis rotation)
  double sinr = +2.0 * (w * x + y * z);
  double cosr = +1.0 - 2.0 * (x * x + y * y);
  roll = atan2(sinr, cosr) * (180/PI);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (w * y - z * x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp) * (180/PI);                      //use 90 degrees if out of range
  else
    pitch = asin(sinp) * (180/PI);

  // yaw (z-axis rotation)
  double siny = +2.0 * (w * z + x * y);
  double cosy = +1.0 - 2.0 * (y * y + z * z);  
  yaw = atan2(siny, cosy) * (180/PI);
  

  /*

  pitch = (uint16_t)((-yaw + angle) * unit + pitch_offset);
  roll = (uint16_t)((-roll + angle) * unit + roll_offset);
  
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
  motor_roll.goalPosition(roll);*/

  pub_pitch.data = pitch;
  pubPitch.publish(&pub_pitch);
  delay(10);

  pub_roll.data = roll;
  pubRoll.publish(&pub_roll);
  delay(10);

  pub_yaw.data = yaw;
  pubYaw.publish(&pub_yaw);
  delay(10);

  nh.spinOnce();
}
