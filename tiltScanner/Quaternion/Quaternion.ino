#define USE_USBCON

#include <ros.h>
#include <ArduinoHardware.h>

#include <DynamixelMotor.h>

#include <geometry_msgs/Quaternion.h>
#include <std_msgs/UInt16.h>

#include <Wire.h>


#define RX 9
#define TX 10
#define CP 5

//uint8_t situation = 1;
uint16_t result_roll = 1;
uint16_t result_pitch = 1;
uint16_t dynamixelPosition = 0;

// --setting for sensor--
const float pitch_offset = 0.0;                                       //Dynamixel unit 1 = 0,088°
const float roll_offset = 0.0;
const float angle = 180.0;                                            //unit = degree
const float unit = 11.377778;


// --settings for Dynamixel--
const uint8_t id_pitch = 2;
const uint8_t id_roll = 1;
const uint32_t baudrate = 57142;
const uint8_t serviceT = 10;

uint16_t approachSpeed = 150;
const static uint16_t levelSpeed = 1024;

const uint16_t homePositionPitch = 2048;
const uint16_t minValuePitch = 1140;
const uint16_t maxValuePitch = 2280;

const uint16_t homePositionRoll = 2048;
const uint16_t minValueRoll = 1448;
const uint16_t maxValueRoll = 2648;

double x = 0, y = 0, z = 0, w = 0;
double roll = 0, pitch = 0, yaw = 0;

ros::NodeHandle nh;

// --Subscriber--

void suborientation(const geometry_msgs::Quaternion sub_orientation)
{
  x = sub_orientation.x;
  y = sub_orientation.y;
  z = sub_orientation.z;
  w = sub_orientation.w;
}
ros::Subscriber<geometry_msgs::Quaternion> subOrientation("imu/data", &suborientation);


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


void setup() 
{ 
  // --initialise the servos--

  interface.begin(baudrate);
  delay(serviceT);
  
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


  /*while(result_pitch || result_roll)                                  //wait for receiving home position
  {
    delay(serviceT);
    motor_pitch.read(0x2E, result_pitch);                             //result = 1 --> moving | result = 0 --> standing
    motor_roll.read(0x2E, result_roll);
  }*/
  
  
}

void loop() 
{
  motor_pitch.speed(levelSpeed);                                  //levelling the laserscanner
  motor_roll.speed(levelSpeed);
  

  // roll (x-axis rotation)
  double sinr = +2.0 * (w * x + y * z);
  double cosr = +1.0 - 2.0 * (x * x + y * y);
  roll = atan2(sinr, cosr) * (180/PI);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (w * y - z * x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp) * (180/PI); // use 90 degrees if out of range
  else
    pitch = asin(sinp) * (180/PI);

  // yaw (z-axis rotation)
  double siny = +2.0 * (w * z + x * y);
  double cosy = +1.0 - 2.0 * (y * y + z * z);  
  yaw = atan2(siny, cosy) * (180/PI);

  /*pitch = (uint16_t)((-yaw + angle) * unit + pitch_offset);
  roll = (uint16_t)((-roll + angle) * unit + roll_offset);
    
  motor_pitch.goalPosition(pitch);
  motor_roll.goalPosition(roll);*/

  //pub_pitch.data = motor_pitch.currentPosition();                     //publish current pitch angle
  pub_pitch.data = pitch;
  pubPitch.publish(&pub_pitch);
  delay(serviceT);

  //pub_roll.data = motor_roll.currentPosition();                       //publish current roll angle
  pub_roll.data = roll;
  pubRoll.publish(&pub_roll);
  delay(serviceT);

  //pub_state.data = (uint16_t)(situation);                             //publish current state
  pub_yaw.data = yaw;
  pubYaw.publish(&pub_yaw);
  delay(serviceT);
  
  nh.spinOnce();
}
