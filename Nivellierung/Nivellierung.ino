//#define USE_USBCON

//#include <ros.h>
#include <ArduinoHardware.h>

#include <DynamixelMotor.h>

//#include <std_msgs/UInt16.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


#define RX 9
#define TX 10
#define CP 5
#define INT A5
#define RST 12

uint8_t situation = 1;
uint16_t result_roll = 1;
uint16_t result_pitch = 1;
uint16_t dynamixelPosition = 0;

bool startReceived = false;

// --setting for sensor--
const float pitch_offset = 13.0;                                       //Dynamixel unit 1 = 0,088°
const float roll_offset = -16.0;
const float angle = 180.0;                                            //unit = degree
const float unit = 11.377778;
uint16_t reboot = 0;


// --settings for Dynamixel--
const uint8_t id_pitch = 2;
const uint8_t id_roll = 1;
const uint32_t baudrate = 500000;
const uint8_t serviceT = 10;

uint16_t approachSpeed = 150;
const static uint16_t levelSpeed = 180;

const uint16_t homePositionPitch = 2048;
const uint16_t minValuePitch = 1140;
const uint16_t maxValuePitch = 2280;

const uint16_t homePositionRoll = 2048;
const uint16_t minValueRoll = 1648;
const uint16_t maxValueRoll = 2448;

uint16_t pitch = 0, roll = 0;


// --settings of service--
uint16_t dynamixelSpeed = 0;
uint16_t startPosition = 0;
uint16_t endPosition = 0;


/*
ros::NodeHandle nh;

// --Publisher--

std_msgs::UInt16 pub_roll;
ros::Publisher pubRoll("roll", &pub_roll);

std_msgs::UInt16 pub_pitch;
ros::Publisher pubPitch("pitch", &pub_pitch);

std_msgs::UInt16 pub_state;
ros::Publisher pubState("state", &pub_state);
*/


Adafruit_BNO055 bno = Adafruit_BNO055();                              //set ... of sensor

SoftwareDynamixelInterface interface(RX,TX,CP);                       //set (RX,TX,Controlpin)   ---hieß anders mit der alten ardyno Version -DynamixelInterface &interface=*createSoftSerialInterface(RX,TX,CP);

DynamixelMotor motor_pitch(interface, id_pitch);                      //set id of Dynamixel_pitch

DynamixelMotor motor_roll(interface, id_roll);                        //set id of Dynamixel_roll


void setup() {

  interface.begin(baudrate);                                          //set Baudrate
  delay(100);

  //nh.initNode();
  

  // --Publisher--
/*
  nh.advertise(pubRoll);
  nh.advertise(pubPitch);
  nh.advertise(pubState);
*/

  // --initialise the sensor--
Serial.begin(57600);
  bno.begin();
  bno.setMode(0x08);
  bno.setAxisRemap(0x21);
  bno.setAxisSign(0x02);
 // bno.setExtCrystalUse(true);                                         //use external crystal for better accuracy
  delay(100);

  // --initialise the servos--
  
  motor_pitch.init();
  motor_pitch.enableTorque();

  motor_roll.init();
  motor_roll.enableTorque();

  motor_pitch.jointMode(minValuePitch, maxValuePitch);                //choose 'jointMode' to enable 'goalPosition'
  //motor_pitch.speed(approachSpeed);                                   //adjustment of speed of the Dynamixel [0.114rpm]
  motor_pitch.goalPosition(homePositionPitch);                        //drive to home position
  motor_pitch.led(HIGH);
  
  motor_roll.jointMode(minValueRoll, maxValueRoll);
  //motor_roll.speed(approachSpeed);
  motor_roll.goalPosition(homePositionRoll);
  motor_roll.led(HIGH);

  delay(1000);


  /*while(result_pitch || result_roll)                                  //wait for receiving home position
  {
    delay(serviceT);
    motor_pitch.read(0x2E, result_pitch);                             //result = 1 --> moving | result = 0 --> standing
    motor_roll.read(0x2E, result_roll);
    motor_roll.led(HIGH);
  }*/
    motor_pitch.led(LOW);
  motor_roll.led(LOW);

 // motor_pitch.speed(levelSpeed);                                  //levelling the laserscanner
 //motor_roll.speed(levelSpeed);
}

void loop() {


      
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  pitch = (uint16_t)(((float)euler.z()+ 1.5*angle) * unit + pitch_offset);
  roll = (uint16_t)(((float)euler.y() + angle) * unit + roll_offset);
Serial.print(euler.y());
Serial.print(", ");
Serial.print(pitch);
Serial.print(", ");
Serial.println(roll);
 motor_pitch.goalPosition(pitch);
 // motor_roll.goalPosition(roll);
  
  //pub_pitch.data = motor_pitch.currentPosition();                     //publish current pitch angle
  //pub_pitch.data = pitch;
  //pubPitch.publish(&pub_pitch);

  //pub_roll.data = motor_roll.currentPosition();                       //publish current roll angle
  //pub_roll.data = roll;
  //pubRoll.publish(&pub_roll);;

  //pub_state.data = (uint16_t)(situation);                             //publish current state
  //pubState.publish(&pub_state);;
  
  //nh.spinOnce();
}