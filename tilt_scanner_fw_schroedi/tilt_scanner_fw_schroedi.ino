#define USE_USBCON

#include <ros.h>
#include <ArduinoHardware.h>

#include <DynamixelMotor.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <ohm_tilt_scanner_3d/MsgScanParams.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


#define RX 9
#define TX 10
#define CP 5
#define INT A5
#define RST 12

uint8_t situation = 1;

uint16_t result_roll = 1;                                             //Values that shows moving of Dynamixel
uint16_t result_pitch = 1;

uint16_t startT = 0;                                                  //starting condition for failure Timer
const uint16_t standbyT = 1000;                                       //waitingtime in case of homeposition doesnt work or IMU crashed

bool startReceived = false;

// --setting for sensor--
const float pitch_offset = 0.0;                                       //Dynamixel unit 1 = 0,088°
const float roll_offset = 11.0;
const float angle = 180.0;                                            //unit = degree
const float unit = 11.377778;

uint16_t reboot = 0;


// --settings for Dynamixel--
const uint8_t id_pitch = 2;
const uint8_t id_roll = 1;
const uint32_t baudrate = 117647;
const uint8_t serviceT = 10;


uint16_t approachSpeed = 150;
const static uint16_t levelSpeed = 1023;                              //1023 = MaxSpee

const uint16_t homePositionPitch = 2048;
const uint16_t minValuePitch = 1400;
const uint16_t maxValuePitch = 2280;

const uint16_t homePositionRoll = 2048;
const uint16_t minValueRoll = 1448;
const uint16_t maxValueRoll = 2648;

uint16_t pitch = 0, roll = 0;


// --settings of service--
uint16_t dynamixelSpeed = 0;
uint16_t startPosition = 0;
uint16_t endPosition = 0;



ros::NodeHandle nh;

// --Subscriber--

void subscanparams(const ohm_tilt_scanner_3d::MsgScanParams& sub_scanparams)
{
  dynamixelSpeed = sub_scanparams.speed;
  startPosition = sub_scanparams.startPosition;
  endPosition = sub_scanparams.endPosition;
  startReceived = true;
}
ros::Subscriber<ohm_tilt_scanner_3d::MsgScanParams> subScanParams("scanparams", &subscanparams);

void subreboot(const std_msgs::UInt16& sub_reboot)
{
  reboot = sub_reboot.data;
}
ros::Subscriber<std_msgs::UInt16> subReboot("ohmTiltScanner3d/reboot", &subreboot);


// --Publisher--

std_msgs::UInt16 pub_roll;
ros::Publisher pubRoll("roll", &pub_roll);

std_msgs::UInt16 pub_pitch;
ros::Publisher pubPitch("pitch", &pub_pitch);

std_msgs::UInt16 pub_state;
ros::Publisher pubState("state", &pub_state);



Adafruit_BNO055 bno = Adafruit_BNO055();                              //set ... of sensor

SoftwareDynamixelInterface interface(RX,TX,CP);                       //set (RX,TX,Controlpin)   ---hieß anders mit der alten ardyno Version -DynamixelInterface &interface=*createSoftSerialInterface(RX,TX,CP);

DynamixelMotor motor_pitch(interface, id_pitch);                      //set id of Dynamixel_pitch

DynamixelMotor motor_roll(interface, id_roll);                        //set id of Dynamixel_roll


void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(RST, HIGH);

  
  nh.initNode();

  // --Subscriber--
  
  nh.subscribe(subScanParams);
  nh.subscribe(subReboot);


  // --Publisher--

  nh.advertise(pubRoll);
  nh.advertise(pubPitch);
  nh.advertise(pubState);


  // --initialise the sensor--

  bno.begin();
  bno.setMode(0x8);
  bno.setAxisRemap(0x21);
  bno.setAxisSign(0x02);
  delay(100);
  
  pinMode(INT, INPUT);
  pinMode(RST, OUTPUT);


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

  startT = millis();

  while(result_pitch || result_roll)                                  //wait for receiving home position
  {
    delay(serviceT);
    motor_pitch.read(0x2E, result_pitch);                             //result = 1 --> moving | result = 0 --> standing
    motor_roll.read(0x2E, result_roll);

    nh.spinOnce();

    if(millis() - startT >= standbyT)                                 //emergency leave of the loop
    {
      break;
    }
  }
}

void loop() {
 
  switch(situation)
  {
    case 1:
    {
      digitalWrite(LED_BUILTIN, HIGH);

      motor_pitch.led(LOW);
      motor_roll.led(LOW);

      motor_pitch.speed(levelSpeed);                                  //levelling the laserscanner
      motor_roll.speed(levelSpeed);
      
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      pitch = (uint16_t)((-(float)(euler.z()) + 0.5 * angle) * unit + pitch_offset);
      roll = (uint16_t)((-(float)(euler.y()) + angle) * unit + roll_offset);
      
      motor_pitch.goalPosition(pitch);
      motor_roll.goalPosition(roll);
      
      if(startReceived)                                                 
      {
        situation = 2;
      }
    }
    break;
      
    case 2:
    {
      motor_pitch.led(LOW);                                           //set Motorled low, if IMU has interrupted
      motor_roll.led(LOW);
      digitalWrite(LED_BUILTIN, LOW);
      
      motor_pitch.speed(approachSpeed);      
      motor_pitch.goalPosition(startPosition);                        //drive to startposition
      
      delay(serviceT);
      motor_pitch.read(0x2E, result_pitch);                            

      if(!result_pitch)
      {
        situation = 3;
      }
    }
    break;

    case 3:
    {
      motor_pitch.speed(dynamixelSpeed);
      motor_pitch.goalPosition(endPosition);                          //continuous moving to endposition
      
      delay(serviceT);
      motor_pitch.read(0x2E, result_pitch);

      if(!result_pitch)
      {
        situation = 4;
      }      
    }
    break;
    
    case 4:
    {      
      motor_pitch.speed(approachSpeed);
      motor_pitch.goalPosition(pitch);                                //scan complete, drive back to last levellingposition
      
      delay(serviceT);
      motor_pitch.read(0x2E, result_pitch);

      if(!result_pitch)
      {
        startReceived = false;
        situation = 1;
      }
    }
    break;
    default:
    break;
  }


  if(digitalRead(INT) || (bool)reboot
    || pitch <= minValuePitch || pitch >= maxValuePitch
    || roll <= minValueRoll || roll >= maxValueRoll)                   
  {
    startT = millis();
    
    if(digitalRead(INT) || (bool)reboot)
    {
      reset_IMU();
    }
    
    while(pitch <= minValuePitch || pitch >= maxValuePitch
          || roll <= minValueRoll || roll >= maxValueRoll)
    {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      pitch = (uint16_t)((-(float)(euler.z()) + 0.5 * angle) * unit + pitch_offset);
      roll = (uint16_t)((-(float)(euler.y()) + angle) * unit + roll_offset);
      
      if(millis() - startT >= standbyT)
      {
        reset_IMU();
      }
      nh.spinOnce();
    }
  }
  

  //pub_pitch.data = motor_pitch.currentPosition();                     //publish current pitch angle
  pub_pitch.data = pitch;
  pubPitch.publish(&pub_pitch);

  //pub_roll.data = motor_roll.currentPosition();                       //publish current roll angle
  pub_roll.data = roll;
  pubRoll.publish(&pub_roll);

  pub_state.data = (uint16_t)(situation);                             //publish current state
  pubState.publish(&pub_state);
  
  nh.spinOnce();
}



void reset_IMU()                                                      //reset IMU and set Motorled high, if IMU interrupts
{
  const uint8_t failure = 100;
  
  reboot = 0;
  digitalWrite(RST, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  motor_pitch.led(HIGH);
  motor_roll.led(HIGH);
  
  delay(10);
  
  digitalWrite(RST, HIGH);
  
  delay(10);
  
  bno.begin();
  bno.setMode(0x8);
  bno.setAxisRemap(0x21);
  bno.setAxisSign(0x02);

  pub_state.data = failure;                                   //publish current state
  pubState.publish(&pub_state);

  nh.spinOnce();
  
  delay (100);
}

