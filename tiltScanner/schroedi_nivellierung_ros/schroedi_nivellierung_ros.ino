#define USE_USBCON

#include <ArduinoHardware.h>

#include <DynamixelMotor.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


#define RX 9
#define TX 10
#define CP 5


// --setting for sensor--
const float pitch_offset = 0.0;                                       //Dynamixel unit 1 = 0,088°
const float roll_offset = 0.0;
const float angle = 180.0;                                            //unit = degree
const float unit = 11.377778;


// --settings for Dynamixel--
const uint8_t id_pitch = 2;
const uint8_t id_roll = 1;
const uint32_t baudrate = 500000;

uint16_t approachSpeed = 150;
const static uint16_t levelSpeed = 1024;                              //1024 = MaxSpeed

const uint16_t homePositionPitch = 2048;
const uint16_t minValuePitch = 1140;
const uint16_t maxValuePitch = 2280;

const uint16_t homePositionRoll = 2048;
const uint16_t minValueRoll = 1448;
const uint16_t maxValueRoll = 2648;

uint16_t pitch = 0, roll = 0;



ros::NodeHandle nh;

// --Publisher--

std_msgs::UInt16 pub_roll;
ros::Publisher pubRoll("roll", &pub_roll);

std_msgs::UInt16 pub_pitch;
ros::Publisher pubPitch("pitch", &pub_pitch);



Adafruit_BNO055 bno = Adafruit_BNO055();                              //set ... of sensor

SoftwareDynamixelInterface interface(RX,TX,CP);                       //set (RX,TX,Controlpin)   ---hieß anders mit der alten ardyno Version -DynamixelInterface &interface=*createSoftSerialInterface(RX,TX,CP);

DynamixelMotor motor_pitch(interface, id_pitch);                      //set id of Dynamixel_pitch

DynamixelMotor motor_roll(interface, id_roll);                        //set id of Dynamixel_roll


void setup() {

  nh.initNode();

  // --Publisher--

  nh.advertise(pubRoll);
  nh.advertise(pubPitch);
  
  
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

  motor_roll.init();
  motor_roll.enableTorque();

  motor_pitch.jointMode(minValuePitch, maxValuePitch);                //choose 'jointMode' to enable 'goalPosition'
  motor_pitch.speed(approachSpeed);                                   //adjustment of speed of the Dynamixel [0.114rpm]
  motor_pitch.goalPosition(homePositionPitch);                        //drive to home position
  
  motor_roll.jointMode(minValueRoll, maxValueRoll);
  motor_roll.speed(approachSpeed);
  motor_roll.goalPosition(homePositionRoll);

  motor_pitch.speed(levelSpeed);                                      //speed for levelling the laserscanner
  motor_roll.speed(levelSpeed);
  delay(1000);
}

void loop() 
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  pitch = (uint16_t)((-(float)(euler.z()) + angle/2) * unit + pitch_offset);
  roll = (uint16_t)((-(float)(euler.y()) + angle) * unit + roll_offset);

  motor_pitch.goalPosition(pitch);
  motor_roll.goalPosition(roll);

  //pub_pitch.data = motor_pitch.currentPosition();                     //publish current pitch angle
  pub_pitch.data = pitch;
  pubPitch.publish(&pub_pitch);

  //pub_roll.data = motor_roll.currentPosition();                       //publish current roll angle
  pub_roll.data = roll;
  pubRoll.publish(&pub_roll);

  nh.spinOnce();
}
