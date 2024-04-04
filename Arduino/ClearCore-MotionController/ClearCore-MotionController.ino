/*

 * Description:
 *   ClearCore program to offer control of an X-Y-Z gantry of 4 ClearPath motors (X, Y1, Y2, Z) over a serial protocol
 *
 *   Protocol:
 *     Client sends newline-terminated commands, ClearCore responds with newline-terminated responses
 *     which will be "SUCCESS: {message}" on success or "ERROR: {message}" on failure
 *
 *     Example Commands:
 *       'm 1,2,3' moves to x,y,z position 1,2,3 in absolute coordinates (motor counts as integers)
 *       'x 1200'  moves the X axis to position 1200 in absolute coordinates (motor count as an integer)
 *       'y 2400'  moves the Y axis to position 2400 in absolute coordinates (motor count as an integer)
 *       'z 10800' moves the Z axis to position 10800 in absolute coordinates (motor count as an integer)
 *       'p'       returns the current x,y,z position (motor counts as integers)
 *       'h'       performs homing routine in X, Y, and Z
 *       'w'       jog +y by the default Y distance
 *       'd'       jog -y by the default Y distance
 *       'a'       jog -x by the default X distance
 *       'd'       jog +x by the default X distance
 *       'o'       jog up (-z) by the default Z distance
 *       'l'       jog down (+z) by the default Z distance
 *       '0'       vacuum off
 *       '1'       vacuum on
 *
 *     ** Special case: When the soft stop button or e-stop button is pressed, ClearCore responds with 
 *        "STOP: Soft Stop Engaged" or "STOP: E-Stop Engaged" (respectively) and will ignore all commands
 *        sent by Client until the stop is released, at which point the ClearCore will respond with
 *        "GO: Soft Stop Released" or "GO: E-Stop Released" (respectively).
 *
 * Requirements:
 *   1. ClearPath motors must be connected to Connector M-0 through M-3
 *   2. The connected ClearPath motors must be configured through the MSP software
 *      for Step and Direction mode (In MSP select Mode>>Step and Direction).
 *   3. The ClearPath motors must be set to use the HLFB mode "ASG-Position
 *      w/Measured Torque" with a PWM carrier frequency of 482 Hz through the MSP
 *      software (select Advanced>>High Level Feedback [Mode]... then choose
 *      "ASG-Position w/Measured Torque" from the dropdown, make sure that 482 Hz
 *      is selected in the "PWM Carrier Frequency" dropdown, and hit the OK
 *      button).
 *   4. Set the Input Format in MSP for "Step + Direction".
 *
 *   ** Note: Set the Input Resolution in MSP the same as your motor's Positioning
 *      Resolution spec if you'd like the pulses sent by ClearCore to command a
 *      move of the same number of Encoder Counts (meaning a 1:1 ratio).
 *
 * Links:
 *   ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 *   ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 *   ** ClearPath Manual (DC Power): https://www.teknic.com/files/downloads/clearpath_user_manual.pdf
 *   ** ClearPath Manual (AC Power): https://www.teknic.com/files/downloads/ac_clearpath-mc-sd_manual.pdf
 *
 */

#include "ClearCore.h"

// When true, will print debug statements to the serial port
const bool debug = false;

// Define which connector and pin the soft estop will use
#define softStopConnector ConnectorDI6
#define softStopPin CLEARCORE_PIN_DI6
bool softStopped = false;

// Keep track of whether the e-stop has been hit
bool eStopped = false;

// Select the baud rate to match the target serial device
const unsigned long baudRate = 9600;

// Define which motor is connected to which port
#define motorX ConnectorM0
#define motorY1 ConnectorM1
#define motorY2 ConnectorM2
#define motorZ ConnectorM3
MotorDriver *motors[] = { &motorX, &motorY1, &motorY2, &motorZ };
const uint8_t motorCount = 4;

// Specify how many motor counts per inch and mm for X and Y
const float motorCountsPerInch = 6523.748;
const float motorCountsPerMillimeter = 266.276;

// Specify a safe starting position for all axes, slightly away from the hard stops at absolute 0
const int32_t offsetHome = 2000;

// Define the velocity and acceleration limits to be used for each move
// 6400 counts = 1 motor rotation
const int velocityLimit = 240000;      // counts per sec
const int accelerationLimit = 240000;  // counts per sec^2

// Define jog distances (in pulses)
const int jogDistanceX = int(6.0 * motorCountsPerInch);
const int jogDistanceY = int(6.0 * motorCountsPerInch);
const int jogDistanceZ = int(1.0 * motorCountsPerInch);

// Global variables for homing and absolute positions
// Coordinates are negative but the - sign is added in MoveAbsolutePosition function
bool homed = false;
const int minX = 0;
const int maxX = 397000;
const int minY = 0;
const int maxY = 800000;
const int minZ = 0;
const int maxZ = 136000;  //150000; changed by Ian based on not wanting to push on the table any more than necessary

// This code has built-in functionality to automatically clear motor alerts,
// including motor shutdowns. Any uncleared alert will cancel and disallow motion.
// WARNING: enabling automatic alert clearing will clear alerts immediately when
// encountered and return a motor to a state in which motion is allowed. Before
// enabling this functionality, be sure to understand this behavior and ensure
// your system will not enter an unsafe state.
// To enable automatic alert clearing, set alwaysClearAlerts = true
// To disable automatic alert clearing, set alwaysClearAlerts = false
const bool alwaysClearAlerts = false;


/*------------------------------------------------------------------------------
 * Function prototypes
 */

bool processAlerts(MotorDriver *motor);
void printAlerts(MotorDriver *motor);
void clearAlerts(MotorDriver *motor);

bool checkX(int32_t position);
bool checkY(int32_t position);
bool checkZ(int32_t position);

bool move(int32_t val1, MotorDriver *m1, int32_t val2, MotorDriver *m2,
          int32_t val3, MotorDriver *m3, int32_t val4, MotorDriver *m4, bool isAbsolute);
bool move1Axis(int32_t val1, MotorDriver *m1, bool isAbsolute);
bool move2Axis(int32_t val1, MotorDriver *m1, int32_t val2, MotorDriver *m2, bool isAbsolute);
bool move3Axis(int32_t val1, MotorDriver *m1, int32_t val2, MotorDriver *m2,
               int32_t val3, MotorDriver *m3, bool isAbsolute);
bool move4Axis(int32_t val1, MotorDriver *m1, int32_t val2, MotorDriver *m2,
               int32_t val3, MotorDriver *m3, int32_t val4, MotorDriver *m4, bool isAbsolute);

void moveRelativeX(int32_t distance);
void moveRelativeY(int32_t distance);
void moveRelativeZ(int32_t distance);

void moveAbsoluteX(int32_t position);
void moveAbsoluteY(int32_t position);
void moveAbsoluteZ(int32_t position);
void moveAbsoluteXYZ(int32_t xPos, int32_t yPos, int32_t zPos);

void waitForMoveToComplete(MotorDriver *motor);

void printCurrentPosXYZ();

void homeAllAxes();
bool home1Axis(int32_t velocity, MotorDriver *motor);
bool home2Axis(int32_t velocity1, MotorDriver *m1, int32_t velocity2, MotorDriver *m2);

void processCommand(char command);

void softStopEngaged();
void softStopReleased();

void stopAllMotors();
void disableAllMotors();
void enableAllMotors();

int32_t readInt();
char readChar();

void vacuumOff();
void vacuumOn();

bool softStopIsPressed();
bool processSoftStop();
bool processEStop();


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

  if (debug) { Serial.println("Setup has begun"); }

  // Set up the interrupt connector in digital input mode.
  // NOTE: The ClearCore manual clearly shows a pull-up resistor is always present,
  // and the "soft stop" is connected between ground and the I/O pin, so it's normally
  // high and will go low when the button is hit.
  softStopConnector.Mode(Connector::INPUT_DIGITAL);

  // Set the ISR's to be called when the SoftStop button is hit or released.
  // Initially turn the interrupts on (third argument 'true')
  //softStopConnector.InterruptHandlerSet(&softStopEngaged, InputManager::FALLING, true);
  //softStopConnector.InterruptHandlerSet(&softStopReleased, InputManager::RISING, true);

  // Read the Soft Stop button's initial state
  // Although the line is active LOW due to a pull-up resistor, it seems ClearCore either
  // in hardware (with an inverter) or in software (in their firmware API) are inverting
  // that notion - probably to make it easier on developers. So the API responds as if it
  // is active HIGH. The ISR though triggers active LOW. How odd.
  if (softStopIsPressed()) {
    softStopped = true;
  }

  if (debug) {
    Serial.print("softStopped: ");
    Serial.println(softStopped);
  }

  // Set up the soft e-stop on all motors
  motorX.EStopConnector(softStopPin);
  motorY1.EStopConnector(softStopPin);
  motorY2.EStopConnector(softStopPin);
  motorZ.EStopConnector(softStopPin);

  // Configure connector 1 to be a digital output that controls the vacuum valve
  ConnectorIO1.Mode(Connector::OUTPUT_DIGITAL);
  vacuumOn();  // ensure the vacuum is ON during startup

  // Set the input clocking rate. This normal rate is ideal for ClearPath step and direction applications.
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

  // Set all motor connectors to ClearPathMotor (CPM) step and direction mode.
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

  // Configure all four motors
  MotorDriver *motor;
  for (uint8_t i = 0; i < motorCount; i++) {
    motor = motors[i];

    // Set the HLFB mode to bipolar PWM
    motor->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);

    // Set the HFLB carrier frequency to 482 Hz
    motor->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    // Set the maximum velocity
    motor->VelMax(velocityLimit);

    // Set the maximum acceleration
    motor->AccelMax(accelerationLimit);

    // Enable the motor
    motor->EnableRequest(true);

    // Wait for HLFB to assert or for an alert to appear
    while (motor->HlfbState() != MotorDriver::HLFB_ASSERTED && !motor->StatusReg().bit.AlertsPresent) {
      continue;
    }

    // Check if motor alert occurred during enabling, and clear alert if present
    if (!processAlerts(motor)) {
      clearAlerts(motor);
    }
  }

  if (debug) { Serial.println("Startup finished"); }
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

  // Check for soft stop and hard e-stop
  processSoftStop();
  processEStop();

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
 *   none
 *
 * Returns: 
 *   none
 */
void processCommand(char command) {
  int32_t xPos = 0;
  int32_t yPos = 0;
  int32_t zPos = 0;

  if (debug) {
    Serial.print("processCommand: ");
    Serial.println(command);
  }

  // Check state of soft estop and hard estop
  processSoftStop();
  processEStop();

  switch (command) {
    case 's':  // jog -y
      if (!softStopped) { moveRelativeY(-jogDistanceY); }
      break;

    case 'w':  // jog +y
      if (!softStopped) { moveRelativeY(jogDistanceY); }
      break;

    case 'a':  // jog -x
      if (!softStopped) { moveRelativeX(-jogDistanceX); }
      break;

    case 'd':  // jog +x
      if (!softStopped) { moveRelativeX(jogDistanceX); }
      break;

    case 'o':  // jog z
      if (!softStopped) { moveRelativeZ(jogDistanceZ); }
      break;

    case 'l':  // jog -z
      if (!softStopped) { moveRelativeZ(-jogDistanceZ); }
      break;

    case 'h':  // perform homing routine
      if (!softStopped) { homeAllAxes(); }
      break;

    case 'x':  // move to absolute x
      xPos = readInt();
      if (!softStopped) { moveAbsoluteX(xPos); }
      break;

    case 'y':  // move to absolute y
      yPos = readInt();
      if (!softStopped) { moveAbsoluteY(yPos); }
      break;

    case 'z':  // move to absolute z
      zPos = readInt();
      if (!softStopped) { moveAbsoluteZ(zPos); }
      break;

    case 'm':  // move to absolute x,y, and z
      xPos = readInt();
      yPos = readInt();
      zPos = readInt();
      if (!softStopped) { moveAbsoluteXYZ(xPos, yPos, zPos); }
      break;

    case '0':  // turn vacuum off
      if (!softStopped) {
        vacuumOff();
        Serial.println("SUCCESS: vacuumOff");
      }
      break;

    case '1':  // turn vacuum on
      if (!softStopped) {
        vacuumOn();
        Serial.println("SUCCESS: vacuumOn");
      }
      break;

    case 'p':  // print current position
      if (!softStopped) { printCurrentPosXYZ(); }
      break;

    default:
      if (!softStopped) {
        Serial.print("ERROR: Invalid command: ");
        Serial.println(command);
      }
      break;
  }

  // Skip the newline at the end of each command
  readChar();
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


/*------------------------------------------------------------------------------
 * processSoftStop
 *
 *   If the soft stop was just engaged, set global softStopped=true and print a "STOP" message to the serial port
 *   If the soft stop was just released, set global softStopped=false and print a "GO" message to the serial port
 *
 * Parameters:
 *   none
 *
 * Returns: 
 *   true if the soft stop is currently not engaged
 *   false if the soft stop is currently engaged
 */
bool processSoftStop() {
  MotorDriver *motor;
  bool softStopCurrentlyPressed = softStopIsPressed();
  bool softStopWasCleared = false;

  // If the soft e-stop was just pressed, return false to indicate the issue
  if (softStopCurrentlyPressed && !softStopped) {
    softStopped = true;
    Serial.println("STOP: Soft Stop Engaged");
    return false;
  }

  // If soft stop was just released, indicate as much
  if ((softStopped == true) && (!softStopCurrentlyPressed)) {
    softStopped = false;
    softStopWasCleared = true;
  }

  // Check each motor for a soft EStop alert, and process it if found
  for (uint8_t i = 0; i < motorCount; i++) {
    motor = motors[i];
    if (motor->AlertReg().bit.MotionCanceledSensorEStop) {
      // If the soft estop has been released, clear the alert
      if (!softStopCurrentlyPressed) {
        clearAlerts(motor);
        Delay_ms(10);  // Delay to wait for alert to fully clear
        softStopWasCleared = true;
      }
    }
  }

  // If we detected the Soft E-Stop was released, print that to the serial port
  if (softStopWasCleared) {
    Serial.println("GO: Soft Stop Released");
  }

  return true;
}

bool softStopIsPressed() {
  return (!softStopConnector.State());
}

bool processEStop() {
  // If one of the motors had a MotorFaulted alert, a main cause is someone hitting the E-Stop

  bool alertWasCleared = false;

  // Check each motor for a soft EStop alert, and process it if found
  MotorDriver *motor;
  for (uint8_t i = 0; i < motorCount; i++) {
    motor = motors[i];
    // If the E-Stop has been recently pressed,
    if (motor->AlertReg().bit.MotorFaulted) {
      // AND the motor is now active again
      if (motor->HlfbState() == MotorDriver::HLFB_ASSERTED) {
        // THEN clear the fault
        clearAlerts(motor);
        alertWasCleared = true;
      }
      else {
        // If E-Stop was *just* pressed, record it and return false to signal the command did not complete successfully
        if (!eStopped) {
          eStopped = true;
          Serial.println("STOP: E-Stop Engaged");
        }
      }
    }
  }

  if (alertWasCleared) {
    eStopped = false;
    Serial.println("GO: E-Stop Released");
    // Make SURE all the motor alerts had a chance to clear
    Delay_ms(1000);
    for (uint8_t i = 0; i < motorCount; i++) {
      motor = motors[i];
      clearAlerts(motor);
    }
  }

  if (eStopped) {
    return false;
  }

  return true;

}


/*------------------------------------------------------------------------------
 * printAlerts
 *
 *   Prints active alerts to the serial port as an error message
 *
 * Parameters:
 *   motor: A pointer to a ClearCore motor connector
 *
 * Returns: 
 *   none
 */
void printAlerts(MotorDriver *motor) {
  // Print status of alerts as error messages over the serial connection
  Serial.print("ERROR: Alerts present: ");
  if (motor->AlertReg().bit.MotionCanceledInAlert) {
    Serial.print("MotionCanceledInAlert ");
  }
  if (motor->AlertReg().bit.MotionCanceledPositiveLimit) {
    Serial.print("MotionCanceledPositiveLimit ");
  }
  if (motor->AlertReg().bit.MotionCanceledNegativeLimit) {
    Serial.print("MotionCanceledNegativeLimit ");
  }
  if (motor->AlertReg().bit.MotionCanceledSensorEStop) {
    Serial.print("MotionCanceledSensorEStop ");
  }
  if (motor->AlertReg().bit.MotionCanceledMotorDisabled) {
    Serial.print("MotionCanceledMotorDisabled ");
  }
  if (motor->AlertReg().bit.MotorFaulted) {
    Serial.print("MotorFaulted ");
  }
  Serial.println("");
}


/*------------------------------------------------------------------------------
 * clearAlerts
 *
 *   Clears alerts, including motor faults. 
 *   Faults are cleared by cycling enable to the motor.
 *   Alerts are cleared by clearing the ClearCore alert register directly.
 *
 * Parameters:
 *   motor: A pointer to a ClearCore motor connector
 *
 * Returns: 
 *   none
 */
void clearAlerts(MotorDriver *motor) {
  if (motor->AlertReg().bit.MotorFaulted) {
    // If a motor fault is present, clear it by cycling enable
    // Serial.println("Faults present. Cycling motor enable signal to motor to clear faults.");
    motor->EnableRequest(false);
    Delay_ms(10);
    motor->EnableRequest(true);
  }
  // Clear alerts
  motor->ClearAlerts();
}


/*------------------------------------------------------------------------------
 * processAlerts
 *
 *   Looks for alerts and calls clearAlerts(motor) if alwaysClearAlerts is set,
 *   otherwise calls printAlerts(motor) and returns false
 *
 * Parameters:
 *   motor: A pointer to a ClearCore motor connector
 *
 * Returns: 
 *   True if there were no alerts (or if they were automatically cleared)
 *   False otherwise.
 */
bool processAlerts(MotorDriver *motor) {
  // If the soft e-stop is pressed, return false indicating the command was not successful
  if (!processSoftStop()) { return false; }

  // If the hard e-stop is pressed, return false indicating the command was not successful
  if (!processEStop()) { return false; }

  // Check if a motor alert is currently preventing motion
  if (motor->StatusReg().bit.AlertsPresent) {
    // Clear alert if configured to do so
    if (alwaysClearAlerts) {
      if (debug) { Serial.println("Alert present. Automatically clearing it."); }
      clearAlerts(motor);
    } else {
      printAlerts(motor);
      return false;
    }
  }
  return true;
}


/*------------------------------------------------------------------------------
 * moveRelative{X,Y,Z}
 *
 *   Move to "distance" number of step pulses away from the current position
 *   Blocks until move is complete.
 *
 * Parameters:
 *   distance: The distance, in relative step pulses, to move
 *
 * Returns: 
 *   none
 */
void moveRelativeX(int32_t distance) {
  // Reverse the sign because our axis *positive* (pointing away from home position) corresponds to a motor counts *negative*
  if (move1Axis(-distance, &motorX, false)) {
    Serial.println("SUCCESS: moveRelativeX");
  }
}

void moveRelativeY(int32_t distance) {
  // Reverse the sign because our axis *positive* (pointing away from home position) corresponds to *fewer* motor counts
  if (move2Axis(-distance, &motorY1, distance, &motorY2, false)) {
    Serial.println("SUCCESS: moveRelativeY");
  }
}

void moveRelativeZ(int32_t distance) {
  // Keep the sign because UP in Z is positive motor counts
  if (move1Axis(distance, &motorZ, false)) {
    Serial.println("SUCCESS: moveRelativeZ");
  }
}

bool move1Axis(int32_t val1, MotorDriver *m1, bool isAbsolute) {
  return move(val1, m1, 0, NULL, 0, NULL, 0, NULL, isAbsolute);
}

bool move2Axis(int32_t val1, MotorDriver *m1, int32_t val2, MotorDriver *m2, bool isAbsolute) {
  return move(val1, m1, val2, m2, 0, NULL, 0, NULL, isAbsolute);
}

bool move3Axis(int32_t val1, MotorDriver *m1, int32_t val2, MotorDriver *m2,
               int32_t val3, MotorDriver *m3, bool isAbsolute) {
  return move(val1, m1, val2, m2, val3, m3, 0, NULL, isAbsolute);
}

bool move4Axis(int32_t val1, MotorDriver *m1, int32_t val2, MotorDriver *m2,
               int32_t val3, MotorDriver *m3, int32_t val4, MotorDriver *m4, bool isAbsolute) {
  return move(val1, m1, val2, m2, val3, m3, val4, m4, isAbsolute);
}

bool move(int32_t val1, MotorDriver *m1, int32_t val2, MotorDriver *m2,
          int32_t val3, MotorDriver *m3, int32_t val4, MotorDriver *m4, bool isAbsolute) {

  // Process any alerts, returning false if there are alerts that weren't auto-cleared
  if ((m1 != NULL) && !processAlerts(m1)) { return false; }
  if ((m2 != NULL) && !processAlerts(m2)) { return false; }
  if ((m3 != NULL) && !processAlerts(m3)) { return false; }
  if ((m4 != NULL) && !processAlerts(m4)) { return false; }

  // Command the move(s)
  // Note: The move command can return false. Currently we catch the causes later on
  // using waitForMoveToComplete and processAlerts, so we don't look at the return value here.
  if (isAbsolute) {
    if (m1 != NULL) { m1->Move(val1, MotorDriver::MOVE_TARGET_ABSOLUTE); }
    if (m2 != NULL) { m2->Move(val2, MotorDriver::MOVE_TARGET_ABSOLUTE); }
    if (m3 != NULL) { m3->Move(val3, MotorDriver::MOVE_TARGET_ABSOLUTE); }
    if (m4 != NULL) { m4->Move(val4, MotorDriver::MOVE_TARGET_ABSOLUTE); }
  } else {
    if (m1 != NULL) { m1->Move(val1); }
    if (m2 != NULL) { m2->Move(val2); }
    if (m3 != NULL) { m3->Move(val3); }
    if (m4 != NULL) { m4->Move(val4); }
  }

  // Wait for move(s) to finish
  if (m1 != NULL) { waitForMoveToComplete(m1); }
  if (m2 != NULL) { waitForMoveToComplete(m2); }
  if (m3 != NULL) { waitForMoveToComplete(m3); }
  if (m4 != NULL) { waitForMoveToComplete(m4); }

  // Process any alerts, returning false if there are alerts that weren't auto-cleared
  if ((m1 != NULL) && !processAlerts(m1)) { return false; }
  if ((m2 != NULL) && !processAlerts(m2)) { return false; }
  if ((m3 != NULL) && !processAlerts(m3)) { return false; }
  if ((m4 != NULL) && !processAlerts(m4)) { return false; }

  return true;
}


/*------------------------------------------------------------------------------
 * waitForMoveToComplete
 *
 *   Wait for the motor to stop moving and assert HLFB.
 *   Returns without waiting if an alert pops up.
 *
 * Parameters:
 *   motor: A pointer to a ClearCore motor connector
 *
 * Returns: 
 *   none
 */
void waitForMoveToComplete(MotorDriver *motor) {
  // Wait for the move to finish or an alert to be set
  if (debug) { Serial.println("Waiting for move to complete..."); }

  // If the soft estop was pressed in the middle of the move, at that point
  // the machine will start decelerating. During that time, case StepsComplete will be false
  // (it will get set to true only after the decel is finished) and the estop alert will not yet be set (it gets set only after the decel is finished).
  // So one way we can detect this is because during decel, the motor's HlfbState() will be HLFB_HAS_MEASUREMENT.
  // ALSO there's the issue of the hard stop. When it's pressed, StepsComplete will be true, HLFB will be deasserted, and AlertsPresent will be true
  // So we want to wait for either of the following cases to be true before we return:
  //   Motor finished move, all good: (StepsComplete==true), HLFB Asserted, and No Alerts Present
  //   Soft stop was hit, motor finished decel: (StepsComplete==true), HLFB is Asserted, Alert is Set
  //   E-Stop was hit: (StepsComplete==true), HLFB is Deasserted, Alert is Set
  //   Some other alert was raised: (StepsComplete==false), HLFB is Asserted, Alert is Set
  while (true) {
    if ((motor->StepsComplete() == true) && (motor->HlfbState() == MotorDriver::HLFB_ASSERTED) && (!motor->StatusReg().bit.AlertsPresent)) { break; }
    if ((motor->StepsComplete() == true) && (motor->HlfbState() == MotorDriver::HLFB_ASSERTED) && (motor->StatusReg().bit.AlertsPresent)) { break; }
    if ((motor->StepsComplete() == true) && (motor->HlfbState() == MotorDriver::HLFB_DEASSERTED) && (motor->StatusReg().bit.AlertsPresent)) { break; }
    if ((motor->StepsComplete() == false) && (motor->HlfbState() == MotorDriver::HLFB_ASSERTED) && (motor->StatusReg().bit.AlertsPresent)) { break; }

    //Serial.print("StepsComplete: ");
    //Serial.print(motor->StepsComplete());
    //Serial.print(", HLFB: ");
    //Serial.print(motor->HlfbState());
    //Serial.print(", AlertsPresent: ");
    //Serial.println(motor->StatusReg().bit.AlertsPresent);
  }

  if (debug) { Serial.println("Checking to see if alerts are present."); }

  // Stop all motors if there was an alert
  if (motor->StatusReg().bit.AlertsPresent) {
    if (debug) {
      Serial.println("Alert was present. Stopping all motors.");
      printAlerts(motor);
    }
    stopAllMotors();
  }
}

bool checkX(int32_t position) {
  // Ensure move is within range
  if ((position < minX) || (position > maxX)) {
    Serial.print("ERROR: X Position ");
    Serial.print(position);
    Serial.print(" out of range (");
    Serial.print(minX);
    Serial.print(" to ");
    Serial.print(maxX);
    Serial.println(")");
    return false;
  }
  return true;
}

bool checkY(int32_t position) {
  if ((position < minY) || (position > maxY)) {
    Serial.print("ERROR: Y Position ");
    Serial.print(position);
    Serial.print(" out of range (");
    Serial.print(minY);
    Serial.print(" to ");
    Serial.print(maxY);
    Serial.println(")");
    return false;
  }
  return true;
}

bool checkZ(int32_t position) {
  if ((position < minZ) || (position > maxZ)) {
    Serial.print("ERROR: Z Position ");
    Serial.print(position);
    Serial.print(" out of range (");
    Serial.print(minZ);
    Serial.print(" to ");
    Serial.print(maxZ);
    Serial.println(")");
    return false;
  }
  return true;
}

/*------------------------------------------------------------------------------
 * moveAbsolute{X,Y,Z}
 *
 *   Move to absolute "position" in step pulses
 *   Blocks until move is complete.
 *
 * Parameters:
 *   position: The position, in absolute step pulses, to move to
 *
 * Returns: 
 *   none
 */
void moveAbsoluteX(int32_t position) {
  // Ensure move is within range
  if (!checkX(position)) { return; }

  // Reverse the sign because our axis *positive* (pointing away from home position) corresponds to a motor counts *negative*
  if (move1Axis(-position, &motorX, true)) {
    Serial.println("SUCCESS: moveAbsoluteX");
  }
}

void moveAbsoluteY(int32_t position) {
  // Ensure move is within range
  if (!checkY(position)) { return; }

  // Reverse the sign because our axis *positive* (pointing away from home position) corresponds to a motor counts *negative*
  if (move2Axis(-position, &motorY1, position, &motorY2, true)) {
    Serial.println("SUCCESS: moveAbsoluteY");
  }
}

void moveAbsoluteZ(int32_t position) {
  // Ensure move is within range
  if (!checkZ(position)) { return; }

  // Subtract position from Max to invert notion of Z axis
  if (move1Axis(position - maxZ, &motorZ, true)) {
    Serial.println("SUCCESS: moveAbsoluteZ");
  }
}

void moveAbsoluteXYZ(int32_t posX, int32_t posY, int32_t posZ) {
  // Ensure movement is within range
  if (!checkX(posX)) { return; }
  if (!checkY(posY)) { return; }
  if (!checkZ(posZ)) { return; }

  // Invert X and Y, and for Z just subtract maxZ from position to invert notion of Z axis
  if (move4Axis(-posX, &motorX, -posY, &motorY1, posY, &motorY2, posZ - maxZ, &motorZ, true)) {
    Serial.println("SUCCESS: moveAbsoluteXYZ");
  }
}

void stopAllMotors() {
  // Stop all motor movement immediately
  motorX.MoveStopAbrupt();
  motorY1.MoveStopAbrupt();
  motorY2.MoveStopAbrupt();
  motorZ.MoveStopAbrupt();
}

void disableAllMotors() {
  // Disable all motors, which sets up alerts
  motorX.EnableRequest(false);
  motorY1.EnableRequest(false);
  motorY2.EnableRequest(false);
  motorZ.EnableRequest(false);
}

void enableAllMotors() {
  // Enable all motors
  motorX.EnableRequest(true);
  motorY1.EnableRequest(true);
  motorY2.EnableRequest(true);
  motorZ.EnableRequest(true);
}

void homeAllAxes() {

  if ((home1Axis(8000, &motorX)) && (home2Axis(8000, &motorY1, -8000, &motorY2)) && (home1Axis(8000, &motorZ))) {
    Serial.println("SUCCESS: homeAllAxes");
  }
}

bool home1Axis(int32_t velocity, MotorDriver *motor) {
  motor->EnableRequest(false);
  Delay_ms(10);
  motor->EnableRequest(true);

  Delay_ms(500);

  // Start moving towards the hardstop
  motor->MoveVelocity(velocity);

  // Check if any motor alerts occurred during onset of motion, and clear alerts if configured to do so
  if (!processAlerts(motor)) {
    // If faults coult not be cleared, cancel homing routine
    stopAllMotors();
    return false;
  }

  // Delay so HLFB has time to deassert
  Delay_ms(100);

  // Waits for HLFB to assert on the motors, meaning the hardstop has been reached
  while (motor->HlfbState() != MotorDriver::HLFB_ASSERTED) {
    // Check if a motor alert occurred so far, and clear alerts if configured to do so
    if (!processAlerts(motor)) {
      // If faults coult not be cleared, cancel homing routine
      stopAllMotors();
      return false;
    }
  }

  // Stop the velocity move now that the hardstop is reached
  motor->MoveStopAbrupt();

  // Delay so HLFB has time to deassert
  Delay_ms(1000);

  // Check if a motor alert occurred so far, and clear alerts if configured to do so
  if (!processAlerts(motor)) {
    // If faults coult not be cleared, cancel homing routine
    stopAllMotors();
    return false;
  }

  // Zero the motor's reference position after homing to allow for accurate
  // absolute position moves
  motor->PositionRefSet(0);

  return true;
}

bool home2Axis(int32_t velocity1, MotorDriver *m1, int32_t velocity2, MotorDriver *m2) {

  m1->EnableRequest(false);
  Delay_ms(10);
  m1->EnableRequest(true);

  m2->EnableRequest(false);
  Delay_ms(10);
  m2->EnableRequest(true);

  Delay_ms(500);

  // Start moving towards the hardstop
  m1->MoveVelocity(velocity1);
  m2->MoveVelocity(velocity2);

  // Check if any motor alerts occurred during onset of motion, and clear alerts if configured to do so
  if ((!processAlerts(m1)) || (!processAlerts(m2))) {
    // If faults coult not be cleared, cancel homing routine
    stopAllMotors();
    return false;
  }

  // Delay so HLFB has time to de-assert
  Delay_ms(100);

  // Waits for HLFB to assert again on both motors, meaning the hardstops have been reached
  while ((m1->HlfbState() != MotorDriver::HLFB_ASSERTED) || (m2->HlfbState() != MotorDriver::HLFB_ASSERTED)) {
    // Check if any motor alerts occurred so far, and clear alerts if configured to do so
    if ((!processAlerts(m1)) || (!processAlerts(m2))) {
      // If faults coult not be cleared, cancel homing routine
      stopAllMotors();
      return false;
    }
  }

  // Stop the velocity move now that the hardstop is reached
  m1->MoveStopAbrupt();
  m2->MoveStopAbrupt();

  // Delay so HLFB has time to deassert
  Delay_ms(1000);

  // Check if any motor alerts occurred so far, and clear alerts if configured to do so
  if ((!processAlerts(m1)) || (!processAlerts(m2))) {
    // If faults coult not be cleared, cancel homing routine
    stopAllMotors();
    return false;
  }

  // Zero the motor's reference position after homing to allow for accurate
  // absolute position moves
  m1->PositionRefSet(0);
  m2->PositionRefSet(0);

  return true;
}

void vacuumOff() {
  // Turn the vacuum off. The vacuum relay's coil is wired between the IO pin and 24V and the
  // output stage will act as a current sink and turn the vacuum ON only when you set the output LOW (false).
  // Likewise, a HIGH (true) output will turn the vacuum OFF.
  ConnectorIO1.State(true);  // output = true -> valve deactivates, vacuum is interrupted, (piece released)
}

void vacuumOn() {
  // Turn the vacuum on.
  ConnectorIO1.State(false);  // output = false -> valve activates, vacuum is NOT interrupted, ready to pick a piece
}

void printCurrentPosXYZ() {
  // Print the current position over the serial port
  Serial.print("SUCCESS: printCurrentPosXYZ ");
  Serial.print((int32_t)(-motorX.PositionRefCommanded()));  // Invert sign
  Serial.print(", ");
  Serial.print((int32_t)(-motorY1.PositionRefCommanded()));  // Invert sign
  Serial.print(", ");
  Serial.println((int32_t)(maxZ + motorZ.PositionRefCommanded()));  // Normally negative. Add to maxZ to invert
}