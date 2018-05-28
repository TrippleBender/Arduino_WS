#include <ArduinoHardware.h>

#include <DynamixelMotor.h>


#define RX 9
#define TX 10
#define CP 5


// --settings for Dynamixel--
const uint8_t id_pitch = 2;
const uint8_t id_roll = 1;
const uint32_t baudrate = 117647;


SoftwareDynamixelInterface interface(RX,TX,CP);                       //set (RX,TX,Controlpin)   ---hieß anders mit der alten ardyno Version -DynamixelInterface &interface=*createSoftSerialInterface(RX,TX,CP);

DynamixelMotor motor_pitch(interface, id_pitch);                      //set id of Dynamixel_pitch

DynamixelMotor motor_roll(interface, id_roll);                        //set id of Dynamixel_roll


void setup() 
{
  Serial.begin(57600);


  // --initialise the servos--

  interface.begin(baudrate);                                          //set Baudrate of Dynamixel
  delay(100);
  
  motor_pitch.init();
  motor_pitch.enableTorque();

  delay(100);
  
  motor_roll.init();
  motor_roll.enableTorque();

  delay(100);

  motor_pitch.led(HIGH);
  motor_roll.led(HIGH);
}

void loop() 
{
  Serial.print("Pitch: ");
  Serial.print(motor_pitch.currentPosition());
  Serial.print("    Roll: ");
  Serial.println(motor_roll.currentPosition());
}
