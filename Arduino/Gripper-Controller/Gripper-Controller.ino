/*
 * Gripper Controller
 *
 * Description:
 *   Arduino Due program to control a Miranda rotational motor and a Microchip linear encoder over a serial protocol
 * 
 * Compiling:  
 *   To compile, install the tc_lib library by cloning https://github.com/antodom/tc_lib
 *   into your Arduino libraries folder:
 *     Windows: My Documents\Arduino\libraries
 *     OSX:     ~/Documents/Arduino/libraries/
 *
 *   Protocol:
 *     Client sends newline-terminated commands, Arduino responds with newline-terminated responses
 *     which will be "SUCCESS: {message}" on success or "ERROR: {message}" on failure
 *
 *     Commands:
 *       'p'       print the current position (motor counts)
 *       'h'       performs homing routine
 *       'r 12345' move to absolute position (0 to 0xFFFF)
 *       'a'       print the linear encoder's analog output value 
 *       'd'       print the linear encoder's digital output value
 *       '1'       turn the LEDs on
 *       '0'       turn the LEDs off
 *
 * Links:
 *   ** Miranda Motor Documentation: https://www.mouser.com/pdfDocs/201215miranda-manual-ovz20008_rev5.pdf
 *   ** Calibrating the Encoder: https://www.youtube.com/watch?v=rJJacayUpTs&t=410s
 *   ** Encoder Documentation: https://ww1.microchip.com/downloads/aemDocuments/documents/MSLD/ProductDocuments/UserGuides/LX34211-User-Guide-50003295.pdf
 *
 */

#include <Wire.h> # For I2C
#include <stdio.h>
#include <stdlib.h>
#include "tc_lib.h" # Timer library, for measuring PWM pulse width

// When true, will print debug statements to the serial port
const bool debug = false;

// Select the baud rate to match the target serial device
const unsigned long baudRate = 9600;

// Encoder Settings
int encoderAnalogPin = A0; // encoder pin analog input

// Miranda Motor Settings
const uint8_t countsPerDegree = 182;
const uint8_t mirandaAddress = 0x50 >> 1; // JP1 in zero position => 7 bit address of 0x28
#define CMD_RESET 0x01
#define CMD_CALIBRATION_COMPLETE 0x02
#define CMD_IS_MOTOR_MOVING 0x03
#define CMD_GET_SETPOINT_POSITION 0x04
#define CMD_GOTO_ABSOLUTE_POSITION 0x05
#define CMD_GOTO_RELATIVE_POSITION 0x06
#define CMD_GET_FIRMWARE_VERSION 0x1B
#define CMD_WAKE_UP 0x1C
#define CMD_GET_ENCODER_POSITION 0x1E
#define CMD_IS_MOTOR_SLEEPING 0x30

// TC library Settings (for pulse width capture/measurement)
#define CAPTURE_TIME_WINDOW 2500000 // usecs
// NOTE: For TC0 (TC0 and channel 0) the TIOA0 is PB25, which is pin 2 for Arduino DUE. 
// So connect the encoder to pin 2
capture_tc0_declaration();
auto& capturePin2=capture_tc0;

// LED Lights settings
const int leftLEDStripPin = 50;
const int rightLEDStripPin = 52;


/*------------------------------------------------------------------------------
 * Function prototypes
 */
void mirandaWrite(uint8_t address, char command, char* txData, uint8_t numDataBytes);
uint8_t mirandaRead(uint8_t address, char command, char* rxData, uint8_t numDataBytes);
void homeMotor();
void rotateMotorAbsolute(uint16_t motorCounts);
void printMotorPosition();
void printLinearEncoderAnalogValue();
void printLinearEncoderDigitalValue();
void processCommand(char command);
int32_t readInt();
char readChar();


/*------------------------------------------------------------------------------
 * setup
 *
 *   Put setup code here. Runs once
 *
 * Parameters:
 *   none
 *
 * Returns: 
 *   none
 */
void setup() {
  // Start serial communication and wait for it to open properly before continuing
  Serial.begin(baudRate); 
  while (!Serial) {}
  if (debug) { Serial.println("Serial is ready"); }
  
  // Initialize I2C interface (to talk to the Miranda motor)
  Wire.begin();
  if (debug) { Serial.println("I2C is ready"); }

  // Turn on LED lights
  pinMode(rightLEDStripPin, OUTPUT);
  pinMode(leftLEDStripPin, OUTPUT);
  digitalWrite(rightLEDStripPin,HIGH);
  digitalWrite(leftLEDStripPin,HIGH);
  if (debug) { Serial.println("LEDs are ready"); }

  // Configure timer capture unit for capturing encoder pulse width
  capturePin2.config(CAPTURE_TIME_WINDOW);
  if (debug) { Serial.println("Timer/Counter capture unit is ready"); }
}


/*------------------------------------------------------------------------------
 * loop
 *
 *   Runs continuously
 *
 * Parameters:
 *   none
 *
 * Returns: 
 *   none
 */
void loop() {
  char command;

  // If a serial command has arrived, process it.
  if (Serial.available()) {
    // get the command (a single letter)
    command = readChar();
    // if the command is not a letter, skip it (could be a framing issue)
    if (!isAlphaNumeric(command)) {
      Serial.print("ERROR: Invalid command: ");
      Serial.println(command);
      return;
    }
    processCommand(command);
  }
}


/*------------------------------------------------------------------------------
 * processCommand
 *
 *   Process an incoming command character, reading additional bytes if 
 *   required by the command
 *
 * Parameters:
 *   command: A single command character
 *
 * Returns: 
 *   none
 */
void processCommand(char command) {
  static int32_t motorCounts = 0;
  
  switch (command) {
    if (debug) { Serial.print("Received command: "); Serial.println(command); }

    case 'h': // home motor
      homeMotor();
      break;

    case 'r': // rotate to absolute position (CW)
      motorCounts = readInt();
      if ((motorCounts < 0) || (motorCounts > 0xFFFF)) {
        Serial.print("ERROR: motorCounts outside of valid range (0-0xFFFF): ");
        Serial.println(motorCounts);
      }
      else {
        rotateMotorAbsolute((uint16_t)motorCounts);
      }
      break;
    
    case 'p': // print current position (both the setpoint and the encoder position, separated by spaces)
      printMotorPosition();
      break;
    
    case 'a': // print encoder analog value
      printLinearEncoderAnalogValue();
      break;
    
    case 'd': // print encoder digital value (the PWM duty cycle)
      printLinearEncoderDigitalValue();
      break;
      
    case '0': // turn LEDs off
      digitalWrite(rightLEDStripPin,LOW);
      digitalWrite(leftLEDStripPin,LOW);
      Serial.println("SUCCESS: setLightOff");
      break;
      
    case '1': // turn LEDs on
      digitalWrite(rightLEDStripPin,HIGH);
      digitalWrite(leftLEDStripPin,HIGH);
      Serial.println("SUCCESS: setLightOn");
      break;

    default:
      Serial.print("ERROR: Invalid command: ");
      Serial.println(command);
      break;
  }
  
  // Skip the newline at the end of each command
  readChar();
}


/*------------------------------------------------------------------------------
 * mirandaWrite
 *
 *   Write to the Miranda motor over I2C
 *
 * Parameters:
 *   address: The I2C address of the Miranda motor
 *   command: A single char representing which command to write
 *   txData: A pointer to the data byte buffer (usually an array)
 *   numDataBytes: The number of data bytes to write from the buffer for this command
 *
 * Returns: 
 *   none
 */
void mirandaWrite(uint8_t address, char command, char* txData, uint8_t numDataBytes) {
  if (debug) { 
    Serial.print("mirandaWrite txData: ");
    for (int i = 0; i < numDataBytes; i++) {
      Serial.print(txData[i], DEC);
      if (i != numDataBytes-1) {
        Serial.print(", ");
      }
    }
    Serial.println();
  }
  
  // Send command
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write(txData, numDataBytes);
  Wire.endTransmission();  
}


/*------------------------------------------------------------------------------
 * mirandaRead
 *
 *   Read data from the Miranda motor over I2C
 *
 * Parameters:
 *   address: The I2C address of the Miranda motor
 *   command: A single char representing which read command to send
 *   rxData: A pointer to the data byte buffer (usually an array)
 *   numDataBytes: The number of data bytes to read into the buffer for this command
 *
 * Returns: 
 *   the number of bytes read
 */
uint8_t mirandaRead(uint8_t address, char command, char* rxData, uint8_t numDataBytes) {
  // Send command to request information
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.endTransmission();
  // Get response
  Wire.requestFrom(address, numDataBytes);
  uint8_t bytesRead = 0;
  while (bytesRead < numDataBytes) {
    // Wait for data to be available
    while (!Wire.available()) {}
    // Then read the byte
    rxData[bytesRead] = Wire.read(); // *rxData++ = Wire.read();
    bytesRead++;
  }
  
  if (debug) { 
    Serial.print("mirandaRead rxData: ");
    for (int i = 0; i < bytesRead; i++) {
      Serial.print(rxData[i], DEC);
      if (i != bytesRead-1) {
        Serial.print(", ");
      }
    }
    Serial.println();
  }
  
  return bytesRead;
}


/*------------------------------------------------------------------------------
 * homeMotor
 *
 *   Perform a homing sequence on the Miranda motor
 *
 * Parameters:
 *   none
 *
 * Returns: 
 *   none
 */
void homeMotor() {
  char txData[] = {0,0,0,0};
  char rxData[] = {0,0,0,0};
  uint8_t numDataBytes = 0;
  
  // Send the reset command and wait 2000ms for it to complete (datasheet says 25ms but it appears to be an error)
  if (debug) { Serial.println("homeMotor: Sending a reset command..."); }
  mirandaWrite(mirandaAddress, CMD_RESET, txData, numDataBytes);
  delay(2000);
  
  // Check to see if the the motor is sleeping (there's a mode where it sleeps after reset)
  if (debug) { Serial.println("homeMotor: Checking to see if motor is sleeping..."); }
  numDataBytes = 1;
  mirandaRead(mirandaAddress, CMD_IS_MOTOR_SLEEPING, rxData, numDataBytes);
  // Result options: 0 = normal operation (awake), 1 = low power mode, 2 = calibrated low power mode
  if (rxData[0] != 0x00) {
    // Motor is not awake, so wake it up
    numDataBytes = 0;
    mirandaWrite(mirandaAddress, CMD_WAKE_UP, txData, numDataBytes);
    delay(100); // Wait for motor to wake up
  }
  
  // Wait for homing/calibration to complete
  if (debug) { Serial.println("homeMotor: Waiting for homing to complete..."); }
  while (true) {
    // Ask the motor if calibration is complete
    numDataBytes = 1;
    mirandaRead(mirandaAddress, CMD_CALIBRATION_COMPLETE, rxData, numDataBytes);
    // Result options: 0 = calibration in progress, 1 = calibration completed, 2 = calibration failed
    if (rxData[0] == 0x00) {
      // Calibration in progress, continue to wait
      delay(100); // Sleep 100ms before we check again
      continue;
    }
    else if (rxData[0] == 0x01) {
      Serial.println("SUCCESS: homeMotor");
      return;
    }
    else if (rxData[0] == 0x02) {
      Serial.println("ERROR: homeMotor failed!");
      return;
    }
  }
}


/*------------------------------------------------------------------------------
 * rotateMotorAbsolute
 *
 *   Rotate the Miranda motor to an absolute position
 *
 * Parameters:
 *   motorCounts: The position (in motor counts) to rotate to
 *
 * Returns: 
 *   none
 */
void rotateMotorAbsolute(uint16_t motorCounts) {
  // Rotate motor to absolute motorCounts position
  char txData[] = {0,0,0,0};
  char rxData[] = {0,0,0,0};
  uint8_t numDataBytes = 2;
  txData[0] = (motorCounts >> 8) & 0x00FF;
  txData[1] = (motorCounts & 0x00FF);
  mirandaWrite(mirandaAddress, CMD_GOTO_ABSOLUTE_POSITION, txData, numDataBytes);
  delay(25); // Delay 25ms to ensure motor begins responding to command
  
  // Wait for motor to finish moving
  while (true) {
    // Check to see if motor is moving
    numDataBytes = 1;
    mirandaRead(mirandaAddress, CMD_IS_MOTOR_MOVING, rxData, numDataBytes);
    // Result options: 0xFF = moving CCW, 0x00 = stopped, 0x01 = moving CW, otherwise error
    if ( (rxData[0] == 0xFF) || (rxData[0] == 0x01) ) {
      // Movement in progress, continue to wait
      delay(100); // Sleep 100ms before we check again 
      continue;
    }
    else if (rxData[0] == 0x00) {
      // Movement has stopped, success!
      Serial.println("SUCCESS: rotateMotorAbsolute");
      return;
    }
    else {
      // Error!
      Serial.println("ERROR: rotateMotorAbsolute");
      return;
    }
  }
}


/*------------------------------------------------------------------------------
 * printMotorPosition
 *
 *   Print the Miranda motor setpoint (commanded) position and also the encoder (actual)
 *   position, separated by spaces.
 *
 * Parameters:
 *   none
 *
 * Returns: 
 *   none
 */
void printMotorPosition() {
  char rxData[] = {0,0,0,0};
  uint8_t numDataBytes = 0;
  uint16_t setpointPosition = 0x00;
  uint16_t encoderPosition = 0x00;
  
  // First get the setpoint (commanded) position
  numDataBytes = 2;
  mirandaRead(mirandaAddress, CMD_GET_SETPOINT_POSITION, rxData, numDataBytes);
  setpointPosition |= (rxData[0] << 8) & 0xFF00;
  setpointPosition |= rxData[1] & 0x00FF;

  // Next get the encoder (actual) position
  numDataBytes = 2;
  mirandaRead(mirandaAddress, CMD_GET_ENCODER_POSITION, rxData, numDataBytes);
  encoderPosition |= (rxData[0] << 8) & 0xFF00;
  encoderPosition |= rxData[1] & 0x00FF;

  // Finally print the results
  Serial.print("SUCCESS: printMotorPosition ");
  Serial.print(setpointPosition);
  Serial.print(" ");
  Serial.println(encoderPosition);
}


/*------------------------------------------------------------------------------
 * printLinearEncoderAnalogValue
 *
 *   Print the analog value from the linear encoder
 *
 * Parameters:
 *   none
 *
 * Returns: 
 *   none
 */
void printLinearEncoderAnalogValue() {
  uint16_t value = analogRead(encoderAnalogPin);
  Serial.print("SUCCESS: printEncoderAnalogValue ");
  Serial.println(value);
}


/*------------------------------------------------------------------------------
 * printLinearEncoderDigitalValue
 *
 *   Print the digital (pwm duty cycle as centi-percentage) value from the linear encoder
 *
 * Parameters:
 *   none
 *
 * Returns: 
 *   none
 */
void printLinearEncoderDigitalValue() {
  // Print the encoder pulse duty cycle as centi-percentage, so 100% returns 10000 and 12.34% returns 1234
  
  // Read PWM duty ("on time") and period (time from the start of one pulse to the start of the next pulse)
  uint32_t status, duty, period;
  status = capturePin2.get_duty_and_period(duty, period);
  
  // Status code potential values (from tc_lib source code):
  // UNSET=0, SET=1, OVERRUN=2, STOPPED=4
  
  if (status == 0) { // UNSET
    Serial.println("ERROR: printEncoderDigitalValue: timer capture is unset");
  }
  else if (status == 1) { // SET
    // Calculate duty cycle as a centi-percentage (and use long to prevent overflow)
    uint16_t dutyCycle = (long)duty * (long)10000 / (long)period;
    
    Serial.print("SUCCESS: printEncoderDigitalValue ");
    Serial.println(dutyCycle);
  }
  else if (status == 2) { // OVERRUN
    Serial.println("ERROR: printEncoderDigitalValue: timer capture is overrun");
  }
  else if (status == 3) { // STOPPED
    Serial.println("ERROR: printEncoderDigitalValue: timer capture is stopped");
  }
}


/*------------------------------------------------------------------------------
 * readChar
 *
 *   Waits for an incoming serial byte and then returns it
 *
 * Parameters:
 *   none
 *
 * Returns: 
 *   The next character read from the serial port
 */
char readChar() {
  // Block (wait) until a character is available, then read and return it.
  while (!Serial.available()) {}
  return (char)Serial.read();
}


/*------------------------------------------------------------------------------
 * readInt
 *
 *   Waits for incoming serial data and then parses an integer (ignores leading whitespace)
 *
 * Parameters:
 *   none
 *
 * Returns: 
 *   The integer that was read from the serial port
 */
int32_t readInt() {
  // Block (wait) until a number is available, then read and return it.
  while (!Serial.available()) {}  // Wait for serial byte to be available
  return (int32_t)Serial.parseInt();
}
