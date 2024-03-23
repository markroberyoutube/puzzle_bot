/*

 * Description:
 *    Move XYZ_3.8
 *    Added !motorX.StepsComplete() to user seeks home homing routine
 *    Added Verbose On Off to toggle serial port feedback in absolute motion
 *    Fixed issues with precision homing in parallel acatuation Y
 *    Added ability to inquiry what is the latest position commanded succesfully (HLBF asserted) to the motors
 *    Handle motor alerts
 *    Fixed homing and updated count per revolution (8x)
 *    Added homing for each axis with user_seek_home
 *    Added absolute position commands
 *
 * Requirements:
 * 1. A ClearPath motor must be connected to Connector M-0-M-3
 * 2. The connected ClearPath motor must be configured through the MSP software
 *    for Step and Direction mode (In MSP select Mode>>Step and Direction).
 * 3. The ClearPath motor must be set to use the HLFB mode "ASG-Position
 *    w/Measured Torque" with a PWM carrier frequency of 482 Hz through the MSP
 *    software (select Advanced>>High Level Feedback [Mode]... then choose
 *    "ASG-Position w/Measured Torque" from the dropdown, make sure that 482 Hz
 *    is selected in the "PWM Carrier Frequency" dropdown, and hit the OK
 *    button).
 * 4. Set the Input Format in MSP for "Step + Direction".
 *
 * ** Note: Homing is optional, and not required in this operational mode or in
 *    this example. This example makes its first move in the positive direction,
 *    assuming any homing move occurs in the negative direction.
 *
 * ** Note: Set the Input Resolution in MSP the same as your motor's Positioning
 *    Resolution spec if you'd like the pulses sent by ClearCore to command a
 *    move of the same number of Encoder Counts, a 1:1 ratio.
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 * ** ClearPath Manual (DC Power): https://www.teknic.com/files/downloads/clearpath_user_manual.pdf
 * ** ClearPath Manual (AC Power): https://www.teknic.com/files/downloads/ac_clearpath-mc-sd_manual.pdf
 *
 *
 */

#include "ClearCore.h"

// Specifies which motor to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motorX ConnectorM0
#define motorY1 ConnectorM1
#define motorY2 ConnectorM2
#define motorZ ConnectorM3
#define motor ConnectorM0

// Connectors that support digital interrupts are:
// DI-6, DI-7, DI-8, A-9, A-10, A-11, A-12
#define interruptConnector ConnectorDI6

// Select the baud rate to match the target serial device
#define baudRate 9600


// When using COM ports, is the device TTL or RS232?
#define isTtlInputPort false


// Specify which serial interface to use as output.
#define SerialPort ConnectorUsb
// Specify which serial interface to use as input.
#define InputPort ConnectorCOM0

// Specify how many motor counts per in or mm for X and Y
#define Count_inch 6523.748
#define Count_mm 266.276

#define Offset_home 2000 //offset from home

// Define the velocity and acceleration limits to be used for each move
// 6400 pulses = 1 motor rotation
int velocityLimit = 240000;      //10000; // pulses per sec
int accelerationLimit = 240000;  //10000; // pulses per sec^2
int DistX = 36000;
int DistY = 60000;
int DistZ = 8000;
char command;
int i = 0;
unsigned int integerValue=0; 
char incomingByte;
bool Verbose = 1;


// homing and absolute positions - coordinates are negative but the - sign is added in MoveAbsolutePosition function
bool Homed = 0;
int Xmin = 0;
int Xmax = 397000;
int Ymin = 0;
int Ymax = 800000;
int Zmin = 0;
int Zmax = 150000;

//relative A to B coordinates
int A_XYZ[] = { -2000, -20000, -2000 };
int B_XYZ[] = { 4000, 18000, -2000 };


int P [] = {-1, -1, -1, -1};    // store the X, Y1, Y2, Z coordinates when CurrentPosXYZ() is called
int PnPdist = 40000;
int PnPdistZ = 24000;
int PickDelay = 250;
int PlaceDelay = 100; 

// This example has built-in functionality to automatically clear motor alerts,
//	including motor shutdowns. Any uncleared alert will cancel and disallow motion.
// WARNING: enabling automatic alert handling will clear alerts immediately when
//	encountered and return a motor to a state in which motion is allowed. Before
//	enabling this functionality, be sure to understand this behavior and ensure
//	your system will not enter an unsafe state.
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)
#define HANDLE_ALERTS (1)

// Declares user-defined helper functions.
// The definition/implementations of these functions are at the bottom of the sketch.

void PrintAlerts();
void HandleAlertsX();
void HandleAlertsY();
void HandleAlertsZ();
void waitForSerial();
bool MoveDistanceX(int distance);
bool MoveDistanceY(int distance);
bool MoveDistanceZ(int distance);
bool MoveDistanceY(int distance);
void PrintAlertsY();
void keyboard_move(int c);
void BasicMoves(int DX, int DY, int DZ);
void PickPlace(int A_XYZ[], int B_XYZ[]);
void SoftStop();
void homeX();
void homeY();
void homeZ();
bool MoveAbsolutePositionX();
bool MoveAbsolutePositionY();
bool MoveAbsolutePositionZ();
bool MoveAbsolutePositionXY();
int32_t ReadNumber ();
void VacOff();
void VacOn();
void CurrentPosXYZ ();


void setup() {
  // Set up the interrupt connector in digital input mode.
  interruptConnector.Mode(Connector::INPUT_DIGITAL);
  // Set an ISR to be called when the state of the interrupt pin goes from
  // true to false.
  interruptConnector.InterruptHandlerSet(SoftStop, InputManager::FALLING,
                                         false);
  // Put your setup code here, it will only run once:

  // Sets the input clocking rate. This normal rate is ideal for ClearPath
  // step and direction applications.
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

  // Sets all motor connectors into step and direction mode.
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                        Connector::CPM_MODE_STEP_AND_DIR);

  // X-Axis
  //Set the motor's HLFB mode to bipolar PWM
  motorX.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // Set the HFLB carrier frequency to 482 Hz
  motorX.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

  // Sets the maximum velocity for each move
  motorX.VelMax(velocityLimit);

  // Set the maximum acceleration for each move
  motorX.AccelMax(accelerationLimit);

  // Y1-Axis
  //Set the motor's HLFB mode to bipolar PWM
  motorY1.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // Set the HFLB carrier frequency to 482 Hz
  motorY1.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

  // Sets the maximum velocity for each move
  motorY1.VelMax(velocityLimit);

  // Set the maximum acceleration for each move
  motorY1.AccelMax(accelerationLimit);

  // Y2-Axis
  //Set the motor's HLFB mode to bipolar PWM
  motorY2.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // Set the HFLB carrier frequency to 482 Hz
  motorY2.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

  // Sets the maximum velocity for each move
  motorY2.VelMax(velocityLimit);

  // Set the maximum acceleration for each move
  motorY2.AccelMax(accelerationLimit);

  // Z-Axis
  //Set the motor's HLFB mode to bipolar PWM
  motorZ.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // Set the HFLB carrier frequency to 482 Hz
  motorZ.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

  // Sets the maximum velocity for each move
  motorZ.VelMax(velocityLimit);

  // Set the maximum acceleration for each move
  motorZ.AccelMax(accelerationLimit);

  //configure pin 1 to be a digital output that controls the vacuum valve
  ConnectorIO1.Mode(Connector::OUTPUT_DIGITAL);
  ConnectorIO1.State(true);  //output = true -> initialize valve to be off

  // Sets up serial communication and waits up to 5 seconds for a port to open.
  // Serial communication is not required for this example to run.
  Serial.begin(baudRate);
  uint32_t timeout = 5000;
  uint32_t startTime = millis();
  while (!Serial && (millis() - startTime < timeout)) {
    continue;
  }

  // Enables the motor; 
  motorX.EnableRequest(true);
  Serial.println("MotorX Enabled");

  motorY1.EnableRequest(true);
  Serial.println("MotorY1 Enabled");

  motorY2.EnableRequest(true);
  Serial.println("MotorY2 Enabled");

  motorZ.EnableRequest(true);
  Serial.println("MotorZ Enabled");

  // Waits for HLFB to assert
  Serial.println("Waiting for HLFB...");
  while (motorX.HlfbState() != MotorDriver::HLFB_ASSERTED && !motorX.StatusReg().bit.AlertsPresent) {
    continue;
  }
  Serial.println("MotorX HLFB Asserted");

  while (motorY1.HlfbState() != MotorDriver::HLFB_ASSERTED && !motorY1.StatusReg().bit.AlertsPresent) {
    continue;
  }
  Serial.println("MotorY1 HLFB Asserted");

  while (motorY2.HlfbState() != MotorDriver::HLFB_ASSERTED && !motorY2.StatusReg().bit.AlertsPresent) {
    continue;
  }
  Serial.println("MotorY2 HLFB Asserted");

  while (motorZ.HlfbState() != MotorDriver::HLFB_ASSERTED && !motorZ.StatusReg().bit.AlertsPresent) {
    continue;
  }
  Serial.println("MotorZ HLFB Asserted");



  // Check if motor alert occurred during enabling
  // Clear alert if configured to do so
  if (motorX.StatusReg().bit.AlertsPresent) {
    Serial.println("MotorX alert detected.");
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsX();
    } else {
      Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.println("Enabling may not have completed as expected. Proceed with caution.");
    Serial.println();
  } else {
    Serial.println("Motor X Ready");
  }


  // Clear alert if configured to do so
  if (motorY1.StatusReg().bit.AlertsPresent) {
    Serial.println("MotorY1 alert detected.");
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsY();
    } else {
      Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.println("Enabling may not have completed as expected. Proceed with caution.");
    Serial.println();
  } else {
    Serial.println("Motor Y1 Ready");
  }


  // Clear alert if configured to do so
  if (motorY2.StatusReg().bit.AlertsPresent) {
    Serial.println("MotorY2 alert detected.");
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsY();
    } else {
      Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.println("Enabling may not have completed as expected. Proceed with caution.");
    Serial.println();
  } else {
    Serial.println("Motor Y2 Ready");
  }

  // Clear alert if configured to do so
  if (motorZ.StatusReg().bit.AlertsPresent) {
    Serial.println("MotorZ alert detected.");
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsZ();
    } else {
      Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.println("Enabling may not have completed as expected. Proceed with caution.");
    Serial.println();
  } else {
    Serial.println("Motor Z Ready");
  }

  //Serial.println("Press any key to complete initial set up...");
  // waitForSerial();

  // Enable digital interrupts.
  interruptConnector.InterruptEnable(true);

  // Remind user that motors are not homed
  Serial.println("Motors need to be homed. Press h to start homing procedure");

}

char ReadChar() {
  // Block (wait) until a character is available.
  while (!Serial.available()) {}
  return char(Serial.read());
}

void loop() {
  // If a serial command has arrived, process it.
  if (Serial.available()) {
    command = ReadChar(); // get the command (a letter)
    if (!isAlpha(command)) {return;} // if the command is not a letter, skip it
    ReadChar(); // Skip the newline
    keyboard_move(command);
  }
}

void keyboard_move(int c) {
  switch (c) {
    case 97:  // 'a' move left (y)
      MoveDistanceY(DistY);
      break;
    case 100:  // 'd' move right (-y)
      MoveDistanceY(-DistY);
      break;
    case 115:  // 's' move back (+x)
      MoveDistanceX(-DistX);
      break;
    case 119:  // 'w' move forward (-x)
      MoveDistanceX(DistX);
      break;
    case 111:  // 'o'move UP (+z)
      MoveDistanceZ(DistZ);
      break;
    case 108:  // 'l' move DOWN (-z)
      MoveDistanceZ(-DistZ);
      break;
    case 55: // '7' home just the Z axis
      homeZ();
      break;
    case 104:  // 'h' home axis (x,y,z)
      //if (Homed) {
      //  Serial.println("Robot already homed, aborting homing...");
      //  break;
      //}
      Serial.println("Homing X");
      homeX();
      Serial.println("Press enter to continue to homing Y");
      ReadChar(); // Skip the newline
      homeY();
      Serial.println("Press enter to continue to homing Z");
      ReadChar(); // Skip the newline
      homeZ();
      break;


    case 120: // 'x' abs movement
      MoveAbsolutePositionX();
      break;
    case 121: // 'y' abs movement
      MoveAbsolutePositionY();
      break;
    case 122: // 'z' abs movement
      MoveAbsolutePositionZ();
      break;
    case 109: // 'm' abs movement
      MoveAbsolutePositionXY();
      break;

    case 112: // 'p' return current axis position verbose
      CurrentPosXYZ();
      Serial.println((String)"Latest commanded coordinates in X, Y1, Y2, Z [" + P[0] +", "+ P[1] +", "+ P[2] +", "+ P[3]  + "]"); 
      break;

    case 99:  // 'c' move diagonally  (x,y)
      MoveDistanceX(int(12*Count_mm)); // move in a 4:3 aspect ratio
      MoveDistanceY(-int(9*Count_mm)); // move in a 4:3 aspect ratio
      break;
    case 118:  // 'v' move diagonally back (x,y)
      MoveDistanceX(-int(12*Count_mm));
      MoveDistanceY(int(9*Count_mm));
      break;
    case 101: // 'e' toggle Verbose On or Off
      if (Verbose) {
        Serial.println("Toggling Verbose off");
        Verbose=0;
      }
      else {
        Serial.println("Toggling Verbose on");
        Verbose=1;
      }
      break;

    case 48:  // '0' turn vacuum off
      VacOff();
      break;

    case 49:  // '1' turn vacuum on
      VacOn();
      break;

    case 98:  // 'b' pick and place test routine
      // start the test routine with the suction cup on the piece, on the table.
      // interupt the suction, go up, move -8000 in the Y
      //
      ConnectorIO1.State(true);  //output = true -> valve closed, piece released
      delay(PlaceDelay);
      MoveDistanceZ(PnPdistZ);
      //MoveDistanceY(-8000);

      for (int i = 0; i <= 10; i++) {

      // pick, move positive Y, place, move further positive Y, then back, pick and place in puzzle island
      MoveDistanceZ(-PnPdistZ);
      ConnectorIO1.State(false);  //output = false -> piece picked
      delay(PickDelay);                 //wait for vacuum to form, gripping the piece
      MoveDistanceZ(PnPdistZ);
      MoveDistanceY(PnPdist);
      MoveDistanceZ(-PnPdistZ);
      ConnectorIO1.State(true);  //output = true -> valve activates, vacuum is interrupted, (piece released)
      delay(PlaceDelay);                //wait for air to enter, piece to release
      MoveDistanceZ(PnPdistZ);
      ConnectorIO1.State(false);  //output = false -> valve deactivates, vacuum is NOT interrupted, ready to pick a piece
      MoveDistanceY(PnPdist);
      MoveDistanceY(-PnPdist);
      MoveDistanceZ(-PnPdistZ);
      delay(PickDelay);
      MoveDistanceZ(PnPdistZ);
      MoveDistanceY(-PnPdist);
      MoveDistanceZ(-PnPdistZ);
      ConnectorIO1.State(true);  //output = true -> valve activates, vacuum is interrupted, (piece released)
      delay(PlaceDelay);
      MoveDistanceZ(PnPdistZ);
      ConnectorIO1.State(false);  //output = false -> valve deactivates, vacuum is NOT interrupted, ready to pick a piece
      // ends at the piece, just above it, ready to pick

      // pick, move negative Y, place, move further negative Y, then back, pick and place in puzzle island
      MoveDistanceZ(-PnPdistZ);
      ConnectorIO1.State(false);  //output = false -> piece picked
      delay(PickDelay);                 //wait for vacuum to form, gripping the piece
      MoveDistanceZ(PnPdistZ);
      MoveDistanceY(-PnPdist);
      MoveDistanceZ(-PnPdistZ);
      ConnectorIO1.State(true);  //output = true -> valve activates, vacuum is interrupted, (piece released)
      delay(PlaceDelay);                //wait for air to enter, piece to release
      MoveDistanceZ(PnPdistZ);
      ConnectorIO1.State(false);  //output = false -> valve deactivates, vacuum is NOT interrupted, ready to pick a piece
      MoveDistanceY(-PnPdist);
      MoveDistanceY(PnPdist);
      MoveDistanceZ(-PnPdistZ);
      delay(PickDelay);
      MoveDistanceZ(PnPdistZ);
      MoveDistanceY(PnPdist);
      MoveDistanceZ(-PnPdistZ);
      ConnectorIO1.State(true);  //output = true -> valve activates, vacuum is interrupted, (piece released)
      delay(PlaceDelay);
      MoveDistanceZ(PnPdistZ);
      ConnectorIO1.State(false);  //output = false -> valve deactivates, vacuum is NOT interrupted, ready to pick a piece

      // pick, move negative X, place, move further negative X, then back, pick and place in puzzle island
      MoveDistanceZ(-PnPdistZ);
      ConnectorIO1.State(false);  //output = false -> piece picked
      delay(PickDelay);                 //wait for vacuum to form, gripping the piece
      MoveDistanceZ(PnPdistZ);
      MoveDistanceX(-PnPdist);
      MoveDistanceZ(-PnPdistZ);
      ConnectorIO1.State(true);  //output = true -> valve activates, vacuum is interrupted, (piece released)
      delay(PlaceDelay);                //wait for air to enter, piece to release
      MoveDistanceZ(PnPdistZ);
      ConnectorIO1.State(false);  //output = false -> valve deactivates, vacuum is NOT interrupted, ready to pick a piece
      MoveDistanceX(-PnPdist);
      MoveDistanceX(PnPdist);
      MoveDistanceZ(-PnPdistZ);
      delay(PickDelay);
      MoveDistanceZ(PnPdistZ);
      MoveDistanceX(PnPdist);
      MoveDistanceZ(-PnPdistZ);
      ConnectorIO1.State(true);  //output = true -> valve activates, vacuum is interrupted, (piece released)
      delay(PlaceDelay);
      MoveDistanceZ(PnPdistZ);
      ConnectorIO1.State(false);  //output = false -> valve deactivates, vacuum is NOT interrupted, ready to pick a piece

      // pick, move positive X, place, move further positive X, then back, pick and place in puzzle island
      MoveDistanceZ(-PnPdistZ);
      ConnectorIO1.State(false);  //output = false -> piece picked
      delay(PickDelay);                 //wait for vacuum to form, gripping the piece
      MoveDistanceZ(PnPdistZ);
      MoveDistanceX(PnPdist);
      MoveDistanceZ(-PnPdistZ);
      ConnectorIO1.State(true);  //output = true -> valve activates, vacuum is interrupted, (piece released)
      delay(PlaceDelay);                //wait for air to enter, piece to release
      MoveDistanceZ(PnPdistZ);
      ConnectorIO1.State(false);  //output = false -> valve deactivates, vacuum is NOT interrupted, ready to pick a piece
      MoveDistanceX(PnPdist);
      MoveDistanceX(-PnPdist);
      MoveDistanceZ(-PnPdistZ);
      delay(PickDelay);
      MoveDistanceZ(PnPdistZ);
      MoveDistanceX(-PnPdist);
      MoveDistanceZ(-PnPdistZ);
      ConnectorIO1.State(true);  //output = true -> valve activates, vacuum is interrupted, (piece released)
      delay(PlaceDelay);
      MoveDistanceZ(PnPdistZ);
      ConnectorIO1.State(false);  //output = false -> valve deactivates, vacuum is NOT interrupted, ready to pick a piece
      // ends at the piece, just above it, ready to pick

      Serial.println("valve test complete");
      SerialPort.SendLine(c);
      }
      break;

    default:
      Serial.println("Mmm.. not sure about that");
      SerialPort.SendLine(c);
      break;
  }
}


// Routine moves
void BasicMoves(int DX, int DY, int DZ) {

  MoveDistanceX(DX);
  MoveDistanceY(DY);

  MoveDistanceX(-DX);
  MoveDistanceX(-DX);
  MoveDistanceX(-DX);

  MoveDistanceY(DY);
  MoveDistanceX(DX);
  MoveDistanceX(DX);
  MoveDistanceY(-2 * DY);
  //delay(2000);

  Serial.println("Press enter to continue...");
  ReadChar(); // Skip the newline
}

void PickPlace() {
  // relative motion pick&place
  MoveDistanceZ(-A_XYZ[2]);
  MoveDistanceX(A_XYZ[0]);
  MoveDistanceY(A_XYZ[1]);
  MoveDistanceZ(A_XYZ[2]);

  MoveDistanceZ(-A_XYZ[2]);
  MoveDistanceX(-A_XYZ[0]);
  MoveDistanceY(-A_XYZ[1]);
  MoveDistanceZ(A_XYZ[2]);
}



/*------------------------------------------------------------------------------
 * PrintAlerts
 *
 *    Prints active alerts.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
void PrintAlerts() {
  // report status of alerts
  Serial.println("Alerts present: ");
  if (motor.AlertReg().bit.MotionCanceledInAlert) {
    Serial.println("    MotionCanceledInAlert ");
  }
  if (motor.AlertReg().bit.MotionCanceledPositiveLimit) {
    Serial.println("    MotionCanceledPositiveLimit ");
  }
  if (motor.AlertReg().bit.MotionCanceledNegativeLimit) {
    Serial.println("    MotionCanceledNegativeLimit ");
  }
  if (motor.AlertReg().bit.MotionCanceledSensorEStop) {
    Serial.println("    MotionCanceledSensorEStop ");
  }
  if (motor.AlertReg().bit.MotionCanceledMotorDisabled) {
    Serial.println("    MotionCanceledMotorDisabled ");
  }
  if (motor.AlertReg().bit.MotorFaulted) {
    Serial.println("    MotorFaulted ");
  }
}
//------------------------------------------------------------------------------


/*------------------------------------------------------------------------------
 * HandleAlerts
 *
 *    Clears alerts, including motor faults. 
 *    Faults are cleared by cycling enable to the motor.
 *    Alerts are cleared by clearing the ClearCore alert register directly.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
void HandleAlertsX() {
  if (motorX.AlertReg().bit.MotorFaulted) {
    // if a motor fault is present, clear it by cycling enable
    Serial.println("Faults present. Cycling motor X enable signal to motor to clear faults.");
    motorX.EnableRequest(false);
    Delay_ms(10);
    motorX.EnableRequest(true);
  }
  // clear alerts
  Serial.println("Clearing alerts.");
  motorX.ClearAlerts();
}
//------------------------------------------------------------------------------

void HandleAlertsZ() {
  if (motorZ.AlertReg().bit.MotorFaulted) {
    // if a motor fault is present, clear it by cycling enable
    Serial.println("Faults present. Cycling motor Z enable signal to motor to clear faults.");
    motorZ.EnableRequest(false);
    Delay_ms(10);
    motorZ.EnableRequest(true);
  }
  // clear alerts
  Serial.println("Clearing alerts.");
  motorZ.ClearAlerts();
}
//------------------------------------------------------------------------------

void HandleAlertsY() {
  // for each motor, if a motor fault is present, clear it by cycling enable
  if (motorY1.AlertReg().bit.MotorFaulted) {
    SerialPort.SendLine("Faults present on motorY1. Cycling enable signal to motor to clear faults.");
    motorY1.EnableRequest(false);
    Delay_ms(10);
    motorY1.EnableRequest(true);
  }
  if (motorY2.AlertReg().bit.MotorFaulted) {
    SerialPort.SendLine("Faults present on motorY2. Cycling enable signal to motor to clear faults.");
    motorY2.EnableRequest(false);
    Delay_ms(10);
    motorY2.EnableRequest(true);
  }
  // clear alerts
  SerialPort.SendLine("Clearing alerts on both motors.");
  motorY1.ClearAlerts();
  motorY2.ClearAlerts();
}
//------------------------------------------------------------------------------

int32_t ReadNumber () {
  while (!Serial.available()) {} // Wait for serial data to be available
  integerValue = 0;		  // throw away previous integerValue
  while(1) {			  // force into a loop until 'n' is received
    incomingByte = Serial.read();
    if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
    if (incomingByte == -1) continue;  // if no characters are in the buffer read() returns -1
    integerValue *= 10;  // shift left 1 decimal place
    // convert ASCII to integer, add, and shift left 1 decimal place
    integerValue = ((incomingByte - 48) + integerValue);
  }
  return integerValue;
}


/*------------------------------------------------------------------------------
 * MoveDistance
 *
 *    Command "distance" number of step pulses away from the current position
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    position)
 *
 * Parameters:
 *    int distance  - The distance, in step pulses, to move
 *
 * Returns: True/False depending on whether the move was successfully triggered.
 */





bool MoveDistanceX(int distance) {
  // Check if a motor alert is currently preventing motion
  // Clear alert if configured to do so
  if (motorX.StatusReg().bit.AlertsPresent) {
    if (Verbose) Serial.println("Motor X alert detected.");
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsX();
    } else {
      Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.println("Move X canceled.");
    Serial.println();
    return false;
  }

  if (Verbose) {
    Serial.print("Moving distance: ");
    Serial.println(distance);
  }

  // Command the move of incremental distance
  motorX.Move(distance);

  // Waits for HLFB to assert (signaling the move has successfully completed)
  if (Verbose) Serial.println("Moving.. Waiting for HLFB");
  while ((!motorX.StepsComplete() || motorX.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motorX.StatusReg().bit.AlertsPresent) {
    continue;
  }
  // Check if motor alert occurred during move
  // Clear alert if configured to do so
  if (motorX.StatusReg().bit.AlertsPresent) {
    Serial.println("Motor X alert detected.");
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsX();
    } else {
      Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.println("Motion may not have completed as expected. Proceed with caution.");
    Serial.println();
    return false;
  } else {
    if (Verbose) Serial.println("Move Done");
    return true;
  }
}
//------------------------------------------------------------------------------

bool MoveDistanceZ(int distance) {
  // Check if a motor alert is currently preventing motion
  // Clear alert if configured to do so
  if (motorZ.StatusReg().bit.AlertsPresent) {
    Serial.println("Motor alert detected.");
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsZ();
    } else {
      Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.println("Move Z canceled.");
    Serial.println();
    return false;
  }

  Serial.print("Moving distance: ");
  Serial.println(distance);

  // Command the move of incremental distance
  motorZ.Move(distance);

  // Waits for HLFB to assert (signaling the move has successfully completed)
  Serial.println("Moving.. Waiting for HLFB");
  while ((!motorZ.StepsComplete() || motorZ.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motorZ.StatusReg().bit.AlertsPresent) {
    continue;
  }
  // Check if motor alert occurred during move
  // Clear alert if configured to do so
  if (motorZ.StatusReg().bit.AlertsPresent) {
    Serial.println("Motor Z alert detected.");
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsZ();
    } else {
      Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.println("Motion may not have completed as expected. Proceed with caution.");
    Serial.println();
    return false;
  } else {
    Serial.println("Move Done");
    return true;
  }
}
//------------------------------------------------------------------------------


bool MoveDistanceY(int distance) {
  // Check if a motor alert is currently preventing motion
  // Clear alert if configured to do so
  if (motorY1.StatusReg().bit.AlertsPresent || motorY2.StatusReg().bit.AlertsPresent) {
    SerialPort.SendLine("Motor alert detected.");
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsY();
    } else {
      SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    SerialPort.SendLine("Move Y canceled.");
    SerialPort.SendLine();
    return false;
  }
  SerialPort.Send("Moving distance: ");
  SerialPort.SendLine(distance);
  // Move both motors the same distance.
  motorY1.Move(distance);
  motorY2.Move(-distance);
  // Wait until both motors complete their moves.
  uint32_t lastStatusTime = Milliseconds();
  while ((!motorY1.StepsComplete() || motorY1.HlfbState() != MotorDriver::HLFB_ASSERTED || !motorY2.StepsComplete() || motorY2.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motorY1.StatusReg().bit.AlertsPresent && !motorY2.StatusReg().bit.AlertsPresent) {
    // Periodically print out why the application is waiting.
    if (Milliseconds() - lastStatusTime > 1000) {
      SerialPort.SendLine("Waiting for HLFB to assert on both motors");
      lastStatusTime = Milliseconds();
    }
    // Check if motor alert occurred during move
    // Clear alert if configured to do so
    if (motorY1.StatusReg().bit.AlertsPresent || motorY2.StatusReg().bit.AlertsPresent) {
      motorY1.MoveStopAbrupt();
      motorY2.MoveStopAbrupt();
      SerialPort.SendLine("Motor alert detected.");
      PrintAlerts();
      if (HANDLE_ALERTS) {
        HandleAlertsY();
      } else {
        SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
      }
      SerialPort.SendLine("Motion may not have completed as expected. Proceed with caution.");
      SerialPort.SendLine();
      return false;
    }
  }
  SerialPort.SendLine("Move Done");
  return true;
}
/*------------------------------------------------------------------------------
 * PrintAlerts
 *
 *    Prints active alerts.
 *
 * Parameters:
 *    requires "motorY1" and "motorY2" to be defined as ClearCore motor connectors
 *
 * Returns: 
 *    none
 */
void PrintAlertsY() {
  // report status of alerts on motorY1
  SerialPort.SendLine("Alerts present on motorY1: ");
  if (motorY1.AlertReg().bit.MotionCanceledInAlert) {
    SerialPort.SendLine("    MotionCanceledInAlert ");
  }
  if (motorY1.AlertReg().bit.MotionCanceledPositiveLimit) {
    SerialPort.SendLine("    MotionCanceledPositiveLimit ");
  }
  if (motorY1.AlertReg().bit.MotionCanceledNegativeLimit) {
    SerialPort.SendLine("    MotionCanceledNegativeLimit ");
  }
  if (motorY1.AlertReg().bit.MotionCanceledSensorEStop) {
    SerialPort.SendLine("    MotionCanceledSensorEStop ");
  }
  if (motorY1.AlertReg().bit.MotionCanceledMotorDisabled) {
    SerialPort.SendLine("    MotionCanceledMotorDisabled ");
  }
  if (motorY1.AlertReg().bit.MotorFaulted) {
    SerialPort.SendLine("    MotorFaulted ");
  }

  // report status of alerts on motorY2
  SerialPort.SendLine("Alerts present on motorY2: ");
  if (motorY2.AlertReg().bit.MotionCanceledInAlert) {
    SerialPort.SendLine("    MotionCanceledInAlert ");
  }
  if (motorY2.AlertReg().bit.MotionCanceledPositiveLimit) {
    SerialPort.SendLine("    MotionCanceledPositiveLimit ");
  }
  if (motorY2.AlertReg().bit.MotionCanceledNegativeLimit) {
    SerialPort.SendLine("    MotionCanceledNegativeLimit ");
  }
  if (motorY2.AlertReg().bit.MotionCanceledSensorEStop) {
    SerialPort.SendLine("    MotionCanceledSensorEStop ");
  }
  if (motorY2.AlertReg().bit.MotionCanceledMotorDisabled) {
    SerialPort.SendLine("    MotionCanceledMotorDisabled ");
  }
  if (motorY2.AlertReg().bit.MotorFaulted) {
    SerialPort.SendLine("    MotorFaulted ");
  }
}
//------------------------------------------------------------------------------
/*------------------------------------------------------------------------------
 * HandleAlerts
 *
 *    Clears alerts, including motor faults. 
 *    Faults are cleared by cycling enable to the motor.
 *    Alerts are cleared by clearing the ClearCore alert register directly.
 *
 * Parameters:
 *    requires "motorY1" and "motorY2" to be defined as ClearCore motor connectors
 *
 * Returns: 
 *    none
 */


void SoftStop() {
  motorX.MoveStopAbrupt();
  motorY1.MoveStopAbrupt();
  motorY2.MoveStopAbrupt();
  motorZ.MoveStopAbrupt();
  SerialPort.SendLine("Soft stop engaged..");
  Serial.print("Press enter to continue...");
  ReadChar(); // Skip the newline
}

void homeY () {
    motorY1.EnableRequest(false);
    Delay_ms(10);
    motorY1.EnableRequest(true);
    motorY2.EnableRequest(false);
    Delay_ms(10);
    motorY2.EnableRequest(true);
    Delay_ms(500);
// Commands a speed of 5000 pulses/sec towards the hardstop for 2 seconds
    SerialPort.SendLine("Moving toward hardstop... Waiting for HLFB");
    motorY1.MoveVelocity(8000);
    motorY2.MoveVelocity(-8000);
    Delay_ms(2000);
    // Then slows down to 1000 pulses/sec until clamping into the hard stop
    motorY1.MoveVelocity(8000);
    motorY2.MoveVelocity(-8000);
    
    // Check if an alert occurred during motion
    if ((motorY1.StatusReg().bit.AlertsPresent)|| (motorY2.StatusReg().bit.AlertsPresent)) {
        // In this case, we can't proceed with homing. Print the alert and bail.
        SerialPort.SendLine("Motor Y1 or Y2 alert occurred during motion. Homing canceled.");
        // The end...
       
    }   


     // Delay so HLFB has time to deassert
    Delay_ms(10);
    // Waits for HLFB to assert again, meaning the hardstop has been reached
    while ((motorY1.HlfbState() != MotorDriver::HLFB_ASSERTED)||(motorY2.HlfbState() != MotorDriver::HLFB_ASSERTED))  {
        if (motorY1.StatusReg().bit.AlertsPresent) {
            // In this case, we can't proceed with homing. Print the alert and bail.
            SerialPort.SendLine("Motor Y1 alert detected. Homing canceled.");
            // The end...
            
        }
        if (motorY2.StatusReg().bit.AlertsPresent) {
            // In this case, we can't proceed with homing. Print the alert and bail.
            SerialPort.SendLine("Motor Y2 alert detected. Homing canceled.");
            // The end...
            
        }
    }

    
    // Stop the velocity move now that the hardstop is reached
    motorY2.MoveStopAbrupt();
    motorY1.MoveStopAbrupt();
    SerialPort.SendLine("Y Hard stop detected");
    Delay_ms(1000);

    // Move away from the hard stop. Any move away from the hardstop will
    // conclude the homing sequence.
    motorY1.Move(-Offset_home);
    motorY2.Move(Offset_home);
    // Delay so HLFB has time to deassert
    Delay_ms(1000);
    // Waits for HLFB to assert, meaning homing is complete
    SerialPort.SendLine("Moving away from hardstop... Waiting for HLFB");
    while ((motorY1.HlfbState() != MotorDriver::HLFB_ASSERTED||!motorY1.StepsComplete())||(motorY2.HlfbState() != MotorDriver::HLFB_ASSERTED||!motorY2.StepsComplete())) {
        continue;
    }
    
    // Check if an alert occurred during offset move
    if ((motorY1.StatusReg().bit.AlertsPresent)|| (motorY2.StatusReg().bit.AlertsPresent)) {
        // In this case, we can't proceed with homing. Print the alert and bail.
        SerialPort.SendLine("Motors Y alert occurred during offset move. Homing canceled.");
        // The end...
        
    } else {
        SerialPort.SendLine("Homing Y Complete. Motor Ready.");
    }
    // Zero the motor's reference position after homing to allow for accurate
    // absolute position moves
    motorY1.PositionRefSet(0);
    motorY2.PositionRefSet(0);

}

void homeX () {
    motorX.EnableRequest(false);
    Delay_ms(10);
    motorX.EnableRequest(true);
// Commands a speed of 5000 pulses/sec towards the hardstop for 2 seconds
    SerialPort.SendLine("Moving toward hardstop... Waiting for HLFB");
    motorX.MoveVelocity(10000);
    
    Delay_ms(2000);
    // Then slows down to 1000 pulses/sec until clamping into the hard stop
    motorX.MoveVelocity(10000);
   
    
    // Check if an alert occurred during motion
    if (motorX.StatusReg().bit.AlertsPresent) {
        // In this case, we can't proceed with homing. Print the alert and bail.
        SerialPort.SendLine("Motor X alert occurred during motion. Homing canceled.");
        // The end...
        
    }   


     // Delay so HLFB has time to deassert
    Delay_ms(10);
    // Waits for HLFB to assert again, meaning the hardstop has been reached
    while (motorX.HlfbState() != MotorDriver::HLFB_ASSERTED)  {
        if (motorX.StatusReg().bit.AlertsPresent) {
            // In this case, we can't proceed with homing. Print the alert and bail.
            SerialPort.SendLine("Motor X alert detected. Homing canceled.");
            // The end...
            
        }
        
    }

    
    // Stop the velocity move now that the hardstop is reached
    motorX.MoveStopAbrupt();
    Delay_ms(1);
    // Move away from the hard stop. Any move away from the hardstop will
    // conclude the homing sequence.
    motorX.Move(-Offset_home);
    
    // Delay so HLFB has time to deassert
    Delay_ms(10);
    // Waits for HLFB to assert, meaning homing is complete
    SerialPort.SendLine("Moving away from hardstop... Waiting for HLFB");
    while (motorX.HlfbState() != MotorDriver::HLFB_ASSERTED||!motorX.StepsComplete()){
        continue;
    }
    
    // Check if an alert occurred during offset move
    if (motorX.StatusReg().bit.AlertsPresent) {
        // In this case, we can't proceed with homing. Print the alert and bail.
        SerialPort.SendLine("Motors X alert occurred during offset move. Homing canceled.");
        // The end...
        
    } else {
        SerialPort.SendLine("Homing X Complete. Motor Ready.");
    }
    // Zero the motor's reference position after homing to allow for accurate
    // absolute position moves
    motorX.PositionRefSet(0);
    Serial.println((String)"Latest commanded coordinates in X  " + motorX.PositionRefCommanded());
    

}


void homeZ () {
    motorZ.EnableRequest(false);
    Delay_ms(10);
    motorZ.EnableRequest(true);
// Commands a speed of 5000 pulses/sec towards the hardstop for 2 seconds
    SerialPort.SendLine("Moving toward hardstop... Waiting for HLFB");
    motorZ.MoveVelocity(8000);
    
    Delay_ms(2000);
    // Then slows down to 1000 pulses/sec until clamping into the hard stop
    motorZ.MoveVelocity(8000);
   
    
    // Check if an alert occurred during motion
    if (motorZ.StatusReg().bit.AlertsPresent) {
        // In this case, we can't proceed with homing. Print the alert and bail.
        SerialPort.SendLine("Motor Z alert occurred during motion. Homing canceled.");
        // The end...
        
    }   


     // Delay so HLFB has time to deassert
    Delay_ms(10);
    // Waits for HLFB to assert again, meaning the hardstop has been reached
    while (motorZ.HlfbState() != MotorDriver::HLFB_ASSERTED)  {
        if (motorZ.StatusReg().bit.AlertsPresent) {
            // In this case, we can't proceed with homing. Print the alert and bail.
            SerialPort.SendLine("Motor Z alert detected. Homing canceled.");
            // The end...
           
        }
        
    }

    
    // Stop the velocity move now that the hardstop is reached
    motorZ.MoveStopAbrupt();
    
    // Move away from the hard stop. Any move away from the hardstop will
    // conclude the homing sequence.
    motorZ.Move(-Offset_home);
    
    // Delay so HLFB has time to deassert
    Delay_ms(10);
    // Waits for HLFB to assert, meaning homing is complete
    SerialPort.SendLine("Moving away from hardstop... Waiting for HLFB");
    while (motorZ.HlfbState() != MotorDriver::HLFB_ASSERTED||!motorZ.StepsComplete()){
        continue;
    }
    
    // Check if an alert occurred during offset move
    if (motorZ.StatusReg().bit.AlertsPresent) {
        // In this case, we can't proceed with homing. Print the alert and bail.
        SerialPort.SendLine("Motors Z alert occurred during offset move. Homing canceled.");
        // The end...
        
    } else {
        SerialPort.SendLine("Homing Z Complete. Motor Ready.");
    }
    // Zero the motor's reference position after homing to allow for accurate
    // absolute position moves
    motorZ.PositionRefSet(0);
    

}


bool MoveAbsolutePositionX() {
  int32_t position=-1;
  // Ask for X coordinate destination
  if (Verbose) Serial.println((String)"Enter X coordinate in motor counts [0 - " + Xmax + "]"); 
  position = ReadNumber(); // adding the negative sign needed 
  if (position < 0) {
    SerialPort.SendLine("Coordinate should be a positive number (absolute coordinate). Exiting motion..");
    return 0;
  }
  else if (position > Xmax) {
    Serial.println((String)"Coordinate should be less than max = " + Xmax +". Exiting motion..");
    return 0;
  }


  // Check if a motor alert is currently preventing motion
  // Clear alert if configured to do so 
  if (motorX.StatusReg().bit.AlertsPresent) {
    SerialPort.SendLine("Motor X alert detected during abs motion.");       
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsX();
    } else {
      SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    SerialPort.SendLine("Motor X move canceled.");      
    SerialPort.SendLine();
    return false;
  }
    
  if (Verbose) {
    SerialPort.Send("Moving Motor X to absolute position: ");
    SerialPort.SendLine(position);
  }
  // Command the move of absolute distance
  motorX.Move(-position, MotorDriver::MOVE_TARGET_ABSOLUTE);
  // Waits for HLFB to assert (signaling the move has successfully completed)
  if (Verbose) SerialPort.SendLine("Moving.. Waiting for HLFB");
  while ( (!motorX.StepsComplete() || motorX.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
         !motorX.StatusReg().bit.AlertsPresent) {
    continue;
  }
  // Check if motor alert occurred during move
  // Clear alert if configured to do so 
  if (motorX.StatusReg().bit.AlertsPresent) {
    SerialPort.SendLine("Motor X alert detected.");       
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsX();
    } else {
      SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
    }
    SerialPort.SendLine("Motion may not have completed as expected. Proceed with caution.");
    SerialPort.SendLine();
    return false;
  } else {
    if (Verbose) SerialPort.SendLine("Move Done");
    return true;
  }

}


bool MoveAbsolutePositionY() {
int32_t position=-1;
// Ask for X coordinate destination
  if (Verbose) Serial.println((String)"Enter Y coordinate in motor counts [0 - " + Ymax + "]"); 
  position=ReadNumber(); // adding the negative sign needed 
  if (position<0) {
    SerialPort.SendLine("Coordinate should be a positive number (absolute coordinate). Exiting motion..");
    return 0;
  }
  else if (position>Ymax) {
    Serial.println((String)"Coordinate should be less than max = " + Ymax +". Exiting motion..");
    return 0;
  }


// Check if a motor alert is currently preventing motion
    // Clear alert if configured to do so 
    if ((motorY1.StatusReg().bit.AlertsPresent)||(motorY2.StatusReg().bit.AlertsPresent)) {
        SerialPort.SendLine("Motor Y alert detected in abs motion.");       
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlertsY();
        } else {
            SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        SerialPort.SendLine("Motor Y move canceled.");      
        SerialPort.SendLine();
        return false;
    }
    
    if (Verbose) {
      SerialPort.Send("Moving Motor Y to absolute position: ");
      SerialPort.SendLine(position);
    }
    // Command the move of absolute distance
    motorY1.Move(-position, MotorDriver::MOVE_TARGET_ABSOLUTE);
    motorY2.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);
    // Waits for HLFB to assert (signaling the move has successfully completed)
    if (Verbose) SerialPort.SendLine("Moving.. Waiting for HLFB");
    while  ((!motorY1.StepsComplete() || motorY1.HlfbState() != MotorDriver::HLFB_ASSERTED || !motorY2.StepsComplete() || motorY2.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motorY1.StatusReg().bit.AlertsPresent && !motorY2.StatusReg().bit.AlertsPresent) {
        continue;
    }
    // Check if motor alert occurred during move
    // Clear alert if configured to do so 
    if ((motorY1.StatusReg().bit.AlertsPresent)||(motorY2.StatusReg().bit.AlertsPresent)) {
        SerialPort.SendLine("Motor Y alert detected.");       
        PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlertsY();
        } else {
            SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
        }
        SerialPort.SendLine("Motion may not have completed as expected. Proceed with caution.");
        SerialPort.SendLine();
        return false;
    } else {
        if (Verbose) SerialPort.SendLine("Move Done");
        return true;
    }

}


bool MoveAbsolutePositionZ() {
  int32_t position=-1;
  // Ask for X coordinate destination
  if (Verbose) Serial.println((String)"Enter Z coordinate in motor counts [0 - " + Zmax + "]"); 
  position=ReadNumber(); // adding the negative sign needed 
  if (position<0) {
    SerialPort.SendLine("Coordinate should be a positive number (absolute coordinate). Exiting motion..");
    return 0;
  }
  else if (position>Zmax) {
    Serial.println((String)"Coordinate should be less than max = " + Zmax +". Exiting motion..");
    return 0;
  }


  // Check if a motor alert is currently preventing motion
  // Clear alert if configured to do so 
  if (motorZ.StatusReg().bit.AlertsPresent) {
    SerialPort.SendLine("Motor Z alert detected.");       
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsZ();
    } else {
      SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    SerialPort.SendLine("Motor Z move canceled.");      
    SerialPort.SendLine();
    return false;
  }
    
  if (Verbose) {
    SerialPort.Send("Moving Motor Z to absolute position: ");
    SerialPort.SendLine(position);
  }
  // Command the move of absolute distance
  motorZ.Move(-position, MotorDriver::MOVE_TARGET_ABSOLUTE);
  // Waits for HLFB to assert (signaling the move has successfully completed)
  if (Verbose) SerialPort.SendLine("Moving.. Waiting for HLFB");
  while ( (!motorZ.StepsComplete() || motorZ.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
           !motorZ.StatusReg().bit.AlertsPresent) {
    continue;
  }
  // Check if motor alert occurred during move
  // Clear alert if configured to do so 
  if (motorZ.StatusReg().bit.AlertsPresent) {
    SerialPort.SendLine("Motor Z alert detected.");       
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsZ();
    } else {
      SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
    }
    SerialPort.SendLine("Motion may not have completed as expected. Proceed with caution.");
    SerialPort.SendLine();
    return false;
  } else {
    if (Verbose) SerialPort.SendLine("Move Done");
    return true;
  }

}



bool MoveAbsolutePositionXY() {
  int32_t positionX=-1;
  int32_t positionY=-1;
  // Ask for X coordinate destination
  if (Verbose) Serial.println((String)"Enter X coordinate in motor counts [0 - " + Xmax + "]"); 
  positionX = ReadNumber(); // adding the negative sign needed 
  if (positionX < 0) {
    SerialPort.SendLine("Coordinate should be a positive number (absolute coordinate). Exiting motion..");
    return 0;
  }
  else if (positionX > Xmax) {
    Serial.println((String)"Coordinate should be less than max = " + Xmax +". Exiting motion..");
    return 0;
  }
  if (Verbose) Serial.println((String)"Enter Y coordinate in motor counts [0 - " + Ymax + "]"); 
  positionY = ReadNumber(); // adding the negative sign needed 
  if (positionY < 0) {
    SerialPort.SendLine("Coordinate should be a positive number (absolute coordinate). Exiting motion..");
    return 0;
  }
  else if (positionY > Ymax) {
    Serial.println((String)"Coordinate should be less than max = " + Ymax +". Exiting motion..");
    return 0;
  }

  // Check if a motor alert is currently preventing motion
  // Clear alert if configured to do so 
  if ((motorY1.StatusReg().bit.AlertsPresent)||(motorY2.StatusReg().bit.AlertsPresent)||(motorX.StatusReg().bit.AlertsPresent)) {
    SerialPort.SendLine("Motor alert detected in abs X Y motion.");       
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsY();
    } else {
      SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    SerialPort.SendLine("Motor move canceled.");      
    SerialPort.SendLine();
    return false;
  }
    
  if (Verbose) Serial.println((String)"Moving Motor X Y to absolute position" + positionX +", "+ positionY);
  // Command the move of absolute distance
  motorY1.Move(-positionY, MotorDriver::MOVE_TARGET_ABSOLUTE);
  motorY2.Move(positionY, MotorDriver::MOVE_TARGET_ABSOLUTE);
  motorX.Move(-positionX, MotorDriver::MOVE_TARGET_ABSOLUTE);
  // Waits for HLFB to assert (signaling the move has successfully completed)
  if (Verbose) SerialPort.SendLine("Moving.. Waiting for HLFB");
  while  ((!motorY1.StepsComplete() || motorY1.HlfbState() != MotorDriver::HLFB_ASSERTED || !motorY2.StepsComplete() || motorY2.HlfbState() != MotorDriver::HLFB_ASSERTED || !motorX.StepsComplete() || motorX.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motorY1.StatusReg().bit.AlertsPresent && !motorY2.StatusReg().bit.AlertsPresent && !motorX.StatusReg().bit.AlertsPresent) {
    continue;
  }
  // Check if motor alert occurred during move
  // Clear alert if configured to do so 
  if ((motorY1.StatusReg().bit.AlertsPresent)||(motorY2.StatusReg().bit.AlertsPresent)||(motorX.StatusReg().bit.AlertsPresent)) {
    SerialPort.SendLine("Motor X or Y alert detected.");       
    PrintAlerts();
    if (HANDLE_ALERTS) {
      HandleAlertsY();
    } else {
      SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
    }
    SerialPort.SendLine("Motion may not have completed as expected. Proceed with caution.");
    SerialPort.SendLine();
    return false;
    } else {
      if (Verbose) SerialPort.SendLine("Move Done");
      return true;
    }

}

void VacOff() {
  ConnectorIO1.State(true);  //output = true -> valve activates, vacuum is interrupted, (piece released)
}

void VacOn() {
 ConnectorIO1.State(false);  //output = false -> valve deactivates, vacuum is NOT interrupted, ready to pick a piece
}

void CurrentPosXYZ() {
  Serial.println((String)"Latest commanded coordinates in X  " + motorX.PositionRefCommanded());
  Serial.println((String)"Latest commanded coordinates in Y1  " + motorY1.PositionRefCommanded());
  Serial.println((String)"Latest commanded coordinates in Y2  " + motorY2.PositionRefCommanded());
  Serial.println((String)"Latest commanded coordinates in Z  " + motorZ.PositionRefCommanded());

  P[0]=motorX.PositionRefCommanded();
  P[1]=motorY1.PositionRefCommanded();
  P[2]=motorY2.PositionRefCommanded();
  P[3]=motorZ.PositionRefCommanded();

}