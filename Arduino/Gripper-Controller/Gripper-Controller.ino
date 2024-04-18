
// Gripper rotation V6
// Updated duty cycle readouts for PWM encoder digital output for Arduino Due
// updated to support Arduino due
// added RS485 communication
// added incremental motion, and interrogate motor for homing
// added ability to receive the amount of rotation in DEG [0-360] or counts [0-65536]
// Gripper motor rotation driver with Miranda motor
// Commands:
// https://www.mouser.com/pdfDocs/201215miranda-manual-ovz20008_rev5.pdf
// 0x05 for absolute rotation from 0 to 0xFFFF (65536 counts) in 2 bytes

#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include "Arduino.h"
//#include "RS485.h"
#include "tc_lib.h"

/*
//RS485
const uint8_t sendPin  = 4;
const uint8_t deviceID = 1;
RS485 rs485(&Serial, sendPin, deviceID);
*/

// Miranda or Titan motor define LENGTH 240
#define LENGTH 230
#define Count_deg 182

// Arduino Due Duty Cycle readout settings for the TC library
#define CAPTURE_TIME_WINDOW 2500000 // usecs
//IMPORTANT: Take into account that for TC0 (TC0 and channel 0) the TIOA0 is PB25, which is pin 2 for Arduino DUE
capture_tc0_declaration();
auto& capture_pin2=capture_tc0;

byte tx_data[2];
byte rx_data[6];
int dev_addr = 0x50 >> 1; //JP1 in zero position => 7 bit address of 0x28
unsigned long start_micros = 0;
unsigned int sin_array[LENGTH];
unsigned int cos_array[LENGTH];
unsigned int sector_array[LENGTH];
unsigned int pindex = 0;
unsigned int integerValue=0; 
char incomingByte;
char command[2]={0,0};
uint16_t angle = 0;
int i=0;
int encoderPin=A0; // encoder pin analog input
int val=0;        // encoder readout
//const int pwmPin = 6;  // PWM pin where the signal is connected
int dutyCycle = 0;
//lights
int pin_lightL = 50;
int pin_lightR = 52;


// declare functions
void TitanWrite(byte address, byte command, byte* tx_data, int n_bytes);
void TitanRead(int address, byte command, byte* rx_data, int n_bytes);
void home();
int32_t ReadNumber ();
int32_t readInt();
void Rotate(uint16_t a);
void waitForSerial();
int getPWMDutyCycle();
//void waitForRS485();
void keyboard_input(int ki);

  
void setup()
{
  //pinMode(toggle_pin,INPUT);
  
  Serial.begin(115200); 
  uint32_t timeout = 5000;
  uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout) {
    continue;
  }
  delay(2000);
  Serial.println("Hello World");
  Wire.begin();

  //wait for motor startup sequence to finish
  //delay(600);
  //Serial.print("RS485_LIB_VERSION: ");
  //Serial.println(RS485_LIB_VERSION);
  //Serial.println("Press any key to complete initial set up...");

  // turn on lights
  
  pinMode(pin_lightR, OUTPUT);
  pinMode(pin_lightL, OUTPUT);

  digitalWrite(pin_lightR,HIGH);
  digitalWrite(pin_lightL,HIGH);

  capture_pin2.config(CAPTURE_TIME_WINDOW);
  
}



void loop() {
  //if (Serial.available()) {
  // command = char(Serial.read());
  Serial.println("Press a key to continue...");
  waitForSerial();
  while(1) {			  // force into a loop until 'n' is received
          while (!Serial.available()) {}
          command[i] = char(Serial.read());
          if (command[i] == '\n') break;   // exit the while(1), we're done receiving
          if (command[i] == -1) continue;  // if no characters are in the buffer read() returns -1
          i = i + 1;
  }
  i=0;
  keyboard_input(command[0]);
  
}


void keyboard_input(int ki) {
   switch (ki) {
      case '1': // rotate 180
        angle = 0x8001;
        tx_data[0] = (angle >> 8) & 0x00FF;
        tx_data[1] = (angle) & 0x00FF;
        TitanWrite(dev_addr, 0x05, tx_data, 2); // go to 1 full rotation
        break;
      case '0': // rotate  back to 0
        angle = 0x0000;
        tx_data[0] = (angle >> 8) & 0x00FF;
        tx_data[1] = (angle) & 0x00FF;
        TitanWrite(dev_addr, 0x05, tx_data, 2); // go to 1 full rotation
        break;
      case 'i': // is the motor homed?
        TitanRead(dev_addr, 0x02, rx_data, 1);
          if (rx_data[0]==0x00) {
            Serial.println("Calibration in progress..");
          }
          else if (rx_data[0]==0x01) {
            Serial.println("Calibration completed");
          }
          else if (rx_data[0]==0x02) {
            Serial.println("Calibration failed..");
          }
          else {
            Serial.println("Unexpected data received on is-homed request");
          }
          break;
      case 'h': // run homing routine
        home();
        break;
      case 'r': // rotate in counts - absolute motion
        Serial.println("Enter motor abs position in counts [0 - 65536]"); 
        angle=ReadNumber(); //  
        Serial.print("Rotating ");
        Serial.println(angle);
        if (angle<0) {
          Serial.println("Rotation should be a positive number (absolute coordinate). Exiting motion..");
          break;
        }
        else if (angle>65536) {
          Serial.println("Rotation should be less than 65536 counts. Exiting motion..");
          break;
        }
        Rotate(angle);
        break;
      case 'd': // rotate in deg - absolute motion
        Serial.println("Enter motor abs position in deg [0 - 360]"); 
        angle=readInt(); //int32_t readInt()   
        Serial.print("Rotating ");
        Serial.println(angle);
        if (angle<0) {
          Serial.println("Rotation should be a positive number (absolute coordinate). Exiting motion..");
          break;
        }
        else if (angle>360) {
          Serial.println("Rotation should be less than 360 deg. Exiting motion..");
          break;
        }
        angle=angle*65536/360;
        Rotate(angle);
        break;
      case 'o': // rotate counter clockwise 1 deg
        Serial.println("Jogging 1 deg clockwise");
        tx_data[0] = (((uint16_t)(-2*Count_deg)) >> 8) & 0x00FF;
        tx_data[1] = ((uint16_t)(-2*Count_deg)) & 0x00FF;
        TitanWrite(dev_addr, 0x06, tx_data, 2); // 
        break;
      case 'l': // rotate clockwise 1 deg
        Serial.println("Jogging 1 deg counter clockwise");
        tx_data[0] = (((uint16_t)(2*Count_deg)) >> 8) & 0x00FF;
        tx_data[1] = ((uint16_t)(2*Count_deg)) & 0x00FF;
        TitanWrite(dev_addr, 0x06, tx_data, 2); // 
        break;
      
      case 'e': // read the encoder analog input
        val = analogRead(encoderPin);  // read the input pin
         Serial.println((String)"Encoder value - analog (0-1024) = " + val);

      case 'p': // read the encoder digital input
        dutyCycle = getPWMDutyCycle();  // Read PWM duty cycle
         Serial.println((String)"Encoder value - digital (0-10000) = " + dutyCycle);
      
      case 10: //'\n'
        break;

      default:
        Serial.println("\nMmm.. not sure about that");
        waitForSerial();
      break;
    }
}

void home() {
  Serial.println("Resetting motor");
  //send the reset command (0x01)
  TitanWrite(dev_addr, 0x01, tx_data, 0);  
  
  delay(3000);
     
  //read firmware version (cmd: 0x1b, 4 bytes to receive)
  TitanRead(dev_addr, 0x1b, rx_data, 4);
  
  //print received data in human readable form
  Serial.print("FW version: ");
  Serial.print(rx_data[0]);
  Serial.print(".");
  Serial.print(rx_data[1]);
  Serial.print(".");
  Serial.println((rx_data[2] << 8) | rx_data[3]);
  
  //is the motor sleeping? (cmd 0x30)
  TitanRead(dev_addr, 0x1c, rx_data, 1);
  if(rx_data[0] == 0x01)
  {
     //send the wake up command (0x1c)
     TitanWrite(dev_addr, 0x1c, tx_data, 0);   
  }
  
  //wait for homing/calibration to complete
  bool finished_homing = false;
  while(!finished_homing)
  {
     TitanRead(dev_addr, 0x02, rx_data, 1);
     if(rx_data[0] == 0x01)
     {
       Serial.println("Homing complete \r\n");
       finished_homing = true;
     }
     else if(rx_data[0] == 0x02)
     {
      Serial.println("Homing failed!\r\n");
      return;
     }
     delay(100);
  }
  
}

void TitanWrite(byte address, byte command, byte* tx_data, int n_bytes)
{
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write(tx_data, n_bytes);
  Wire.endTransmission();  
}

void TitanRead(int address, byte command, byte* rx_data, int n_bytes)
{
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.endTransmission();
  
  Wire.requestFrom(address, n_bytes);
  while(Wire.available())
  {
      *rx_data++ = Wire.read();
  }  
}




 void Rotate (uint16_t a) {
  Serial.println((String)"Rotating to abs count "+a);
  tx_data[0] = (a >> 8) & 0x00FF;
  tx_data[1] = (a) & 0x00FF;
  TitanWrite(dev_addr, 0x05, tx_data, 2); // go to 1 full rotation
 }


 int32_t ReadNumber () {
  int c=0;
  while (!Serial.available()) {}
  integerValue = 0;		  // throw away previous integerValue
  while(1) {			  // force into a loop until 'n' is received
    while (!Serial.available()) {}
    incomingByte = Serial.read();
    if ((incomingByte == '\n')&&(c)) break;   // exit the while(1), we're done receiving
   // if ((incomingByte == '\n')&&(!c)) continue;   // ignore \n as first char
    if (incomingByte == -1) continue;  // if no characters are in the buffer read() returns -1
    integerValue *= 10;  // shift left 1 decimal place
    // convert ASCII to integer, add, and shift left 1 decimal place
    integerValue = ((incomingByte - 48) + integerValue);
    c++;
  }
  return integerValue;
}

int32_t readInt() {
  // Block (wait) until a number is available, then read and return it.
  while (!Serial.available()) {}  // Wait for serial byte to be available
  return (int32_t)Serial.parseInt();
}

void waitForSerial() {
  while (!Serial.available()) {
  }
  //Serial.println(Serial.read());
}

int getPWMDutyCycle() {
  // Read PWM duty cycle
  uint32_t status,duty,period;
  status=capture_pin2.get_duty_and_period(duty,period);
  
  // Calculate duty cycle as a percentage
  int dc = ((double)duty/period)*10000;
  
  return dc;
}


/*
int getPWMDutyCycle_old(int pin) {
  // Read PWM duty cycle
  unsigned long highTime = pulseIn(pin, HIGH);  // Time in microseconds when the signal is high
  unsigned long lowTime = pulseIn(pin, LOW);    // Time in microseconds when the signal is low
  
  // Calculate duty cycle as a percentage
  int dc = map(highTime, 0, highTime + lowTime, 0, 1000);
  
  return dc;
}
*/


/*
void waitForRS485() {
  while (!rs485.available()) {
  }
  //Serial.println(Serial.read());
}
*/


