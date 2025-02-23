/*
  RavensC++, 2024-2025
  Team ID: 3991-1
  Team Members: Rudra Kumar, Ali Zain Bukhari, Dean Forsyth, Benjamin Leonard, James Rogers, and Caitlyn Volkert.
  Robofest Exhibiton
  Sunlake Academy of Math and Science
  Sphero RVR + Sparkfun Redboard Plus
*/

//Libraries:
#include <Wire.h>                              // Wire/Qwiic
#include <Adafruit_PWMServoDriver.h>           // Servo/PCA9685
#include <SparkFun_I2C_Mux_Arduino_Library.h>  //Qwiic Mux TCA9548A
#include <SFE_MicroOLED.h>                     //Qwiic Mini Display
#include "SparkFun_I2C_GPS_Arduino_Library.h"  //Qwiic GPS XA1110
#include "SparkFun_VL53L1X.h"                  //Qwiic Distance VL531X
#include <SpheroRVR.h>                         //Sphero RVR
#include <SparkFun_Qwiic_Button.h>
#include <Servo.h>  //Qwiic Button

//Variables:
#define PIN_RESET 9

#define SERVOMIN 160   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 500   // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600      // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400     // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

QwiicButton button;
QWIICMUX myMux;
SFEVL53L1X distanceSensor;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//RVR Functions:
static DriveControl driveControl;  //Drive Control
static uint32_t ledGroup;          //LEDs
static LedControl ledControl;
static void getBatteryVoltageStateCallback(GetBatteryVoltageStateReturn_t *batteryVoltageStateReturn);  //Battery
MicroOLED oled(PIN_RESET);                                                                              //OLED

//Button Functions:
uint8_t brightness = 250;   //The maximum brightness of the pulsing LED. Can be between 0 (min) and 255 (max)
uint16_t cycleTime = 1000;  //The total time for the pulse to take. Set to a bigger number for a slower pulse, or a smaller number for a faster pulse
uint16_t offTime = 200;     //The total time to stay off between pulses. Set to 0 to be pulsing continuously.

/*
QWIIC PORTS
Right Distance Port: 4
Front Distance Port: 6
Left Distance Port: 7
Button port : 0
Display port: 1
GPS port: 3

SERVO PORTS
ARMEntry: 0
ARMCenter: 4
ARMShovel: 8
SEEDcontrol: 12
*/


void setup() {
  // set up communication with the RVR
  rvr.configUART(&Serial);
  rvr.wake();
  delay(2000);

  // RVR drive control
  driveControl = rvr.getDriveControl();

  // RVR led control
  ledControl = rvr.getLedControl();

  //Start setup:
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  setupMux();
  setPort(1);

  setupOled();

  delay(2000);  // Delay 1000 ms
  Serial.print("_________________________\n");
  Serial.print("RavensC++ SERIAL MONITIOR\n");

  oledDisplayMessage("RavensC++\nID:3991-1\n \nPush to\nstart\n");

  setPort(0);
  button.begin();
  button.LEDoff();
  while (button.isPressed() == false) {
    button.LEDconfig(brightness, cycleTime, offTime);
    if (button.isPressed() == true) {
      Serial.println("Button Pressed. Starting...");
      button.LEDon(20);
      break;
    }
  }

  //END setup.

  setPort(1);

  oledDisplayMessage("RavensC++\nID:3991-1\n\n\nStarting.. \n");

  Serial.println("after start");
  
  


  /*myMux.setPort(6);
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.stopRanging();

  Serial.print("Distance(mm): ");
  Serial.print(distance);
  if (distance < 100){
    move(1,2000);
    Serial.print("stop");
    stop();
  } else {
    distanceSensor.startRanging();
    int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor.stopRanging();

    Serial.print("Distance(mm): ");
    Serial.print(distance);
  };*/
  /*driveControl.rollStart(0, 1);
  void setDistanceModeShort();
  int frdistance = distanceSensor.getDistance();
  Serial.print(frdistance);
  if (frdistance = distanceSensor.getDistance() < 199) {
    rvr.resetYaw();
    driveControl.setHeading(0);
    driveControl.rollStart(0, 1);
    delay(3000);
    rvr.resetYaw();
    driveControl.setHeading(0);
  };
  if (frdistance = distanceSensor.getDistance() > 199) {
    rvr.resetYaw();
    driveControl.setHeading(0);
    turn(90);
  };*/
  /*myMux.setPort(6);
  distanceSensor.startRanging();                //Write configuration bytes to initiate measurement
  int distance = distanceSensor.getDistance();  //Get the result of the measurement from the sensor

  Serial.print("Distance(mm): ");
  Serial.print(distance);

  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;

  Serial.print("\tDistance(ft): ");
  Serial.print(distanceFeet, 2);

  Serial.println();
  
 if (distanceInches > 2){
    int distance = distanceSensor.getDistance();
    rvr.resetYaw();
    driveControl.setHeading(0);
    driveControl.rollStart(0, 100);
  
    float distanceInches = distance * 0.0393701;
 };

 if(distanceInches <= 2){
  float distanceInches = distance * 0.0393701;
 delay(100);
 };*/

};

void seedSpit() {
  movepositiveServo(3, 90);
  movenegativeServo(3, 30);
}

long resetPulseLength = 0;

void movepositiveServo(int port, int degrees) {
  long pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);

  for (uint16_t pulse = SERVOMIN; pulse < pulselength; pulse++) {

    pwm.setPWM(port, 0, pulse);
    delay(2);
    resetPulseLength = pulse;
  }
}

void movenegativeServo(int port, int degrees) {
  long pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);


  for (uint16_t pulse = resetPulseLength; pulse > pulselength; pulse--) {
    pwm.setPWM(port, 0, pulse);
    delay(2);
  }
}

void oledDisplayMessage(char message[]) {
  oled.clear(PAGE);
  delay(1000);
  oled.setCursor(0, 0);
  oled.print(message);
  oled.display();
}

void setupMux() {
  myMux.begin();
  byte currentPortNumber = myMux.getPort();
}

void setPort(int muxPort) {
  myMux.setPort(muxPort);
}

void setupOled() {
  //OLED
  oled.begin(0x3D, Wire);  // Initialize the OLED
  oled.clear(ALL);         // Clear the display's internal memory
  oled.display();          // Display what's in the buffer (splashscreen)
  oled.setFontType(0);
}



/*void turnmes() {

  distanceSensor.startRanging();
  myMux.setPort(6);
  driveControl.rollStart(0, 100);
  void setDistanceModeShort();
  int distance = distanceSensor.getDistance();
  //Serial.print(distance);
  if (distance = distanceSensor.getDistance() < 199) {

    driveControl.setHeading(0);
    driveControl.rollStart(0, 0);
    delay(2000);
  };
};*/

int turn(uint16_t heading) {
  /*90: left
    180: u-turn
    270: right*/
  rvr.resetYaw();
  driveControl.setHeading(0);
  driveControl.rollStart(heading, 1);
  delay(1000);
  rvr.resetYaw();
  driveControl.setHeading(0);
  driveControl.rollStop(0);
}

void stop() {
  rvr.resetYaw();
  driveControl.setHeading(0);
  driveControl.rollStop(0);
}

int move(int speed, int time) {
  rvr.resetYaw();
  driveControl.setHeading(0);
  driveControl.rollStart(0, 100);
  delay(3000);
  rvr.resetYaw();
  driveControl.setHeading(0);
  driveControl.rollStop(0);
}

void loop(){


};