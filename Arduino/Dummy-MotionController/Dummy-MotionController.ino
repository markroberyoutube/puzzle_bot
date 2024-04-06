#define SOFT_STOP_PIN 2
#define E_STOP_PIN 3

const bool debug = false;

// Specify how many motor counts per inch and mm for X and Y
const float motorCountsPerInch = 6523.748;
const float motorCountsPerMillimeter = 266.276;

// Define jog distances (in pulses)
const int jogDistanceX = int(6.0 * motorCountsPerInch);
const int jogDistanceY = int(6.0 * motorCountsPerInch);
const int jogDistanceZ = int(1.0 * motorCountsPerInch);

bool softStopped = false;
bool eStopped = false;

void setup() {
  // Enable serial port
  Serial.begin(9600);
  while (!Serial) {} // wait for serial to connect. needed for native usb 

  // Setup inputs
  pinMode(SOFT_STOP_PIN, INPUT_PULLUP);
  pinMode(E_STOP_PIN, INPUT_PULLUP);
}

void processSoftStop() {
  if (digitalRead(SOFT_STOP_PIN) == LOW) {
    if (softStopped == false) {
      Serial.println("STOP: Soft Stop Engaged");
      softStopped = true;
    }
  }
  else {
    if (softStopped == true) {
      Serial.println("GO: Soft Stop Released");
      softStopped = false;
    }
  }
}

void processEStop() {
  if (digitalRead(E_STOP_PIN) == LOW) {
    if (eStopped == false) {
      Serial.println("STOP: E-Stop Engaged");
      eStopped = true;
    }
  }
  else {
    if (eStopped == true) {
      Serial.println("GO: E-Stop Released");
      eStopped = false;
    }
  }
}

char readChar() {
  // Block (wait) until a character is available, then read and return it.
  while (!Serial.available()) {}
  return (char)Serial.read();
}

int32_t readInt() {
  // Block (wait) until a number is available, then read and return it.
  while (!Serial.available()) {}  // Wait for serial byte to be available
  return (int32_t)Serial.parseInt();
}

void processCommand(char command) {
  static int32_t xPos = 0;
  static int32_t yPos = 0;
  static int32_t zPos = 0;

  if (debug) {
    Serial.print("processCommand: ");
    Serial.println(command);
  }

  // Check state of soft estop and hard estop
  processSoftStop();
  processEStop();

  switch (command) {
    case 's':  // jog -y
      if (!softStopped) { Serial.println("SUCCESS: moveRelativeY"); yPos -= jogDistanceY; }
      break;

    case 'w':  // jog +y
      if (!softStopped) { Serial.println("SUCCESS: moveRelativeY"); yPos += jogDistanceY; }
      break;

    case 'a':  // jog -x
      if (!softStopped) { Serial.println("SUCCESS: moveRelativeX"); xPos -= jogDistanceX; }
      break;

    case 'd':  // jog +x
      if (!softStopped) { Serial.println("SUCCESS: moveRelativeX"); xPos += jogDistanceX; }
      break;

    case 'o':  // jog z
      if (!softStopped) { Serial.println("SUCCESS: moveRelativeZ"); zPos += jogDistanceZ; }
      break;

    case 'l':  // jog -z
      if (!softStopped) { Serial.println("SUCCESS: moveRelativeZ"); zPos -= jogDistanceZ; }
      break;

    case 'h':  // perform homing routine
      if (!softStopped) { Serial.println("SUCCESS: homeAllAxes"); }
      break;

    case 'x':  // move to absolute x
      xPos = readInt();
      if (!softStopped) { Serial.println("SUCCESS: moveAbsoluteX"); }
      break;

    case 'y':  // move to absolute y
      yPos = readInt();
      if (!softStopped) { Serial.println("SUCCESS: moveAbsoluteY"); }
      break;

    case 'z':  // move to absolute z
      zPos = readInt();
      if (!softStopped) { Serial.println("SUCCESS: moveAbsoluteZ"); }
      break;

    case 'm':  // move to absolute x,y, and z
      xPos = readInt();
      yPos = readInt();
      zPos = readInt();
      if (!softStopped) { Serial.println("SUCCESS: moveAbsoluteXYZ"); }
      break;

    case '0':  // turn vacuum off
      if (!softStopped) { Serial.println("SUCCESS: vacuumOff"); }
      break;

    case '1':  // turn vacuum on
      if (!softStopped) { Serial.println("SUCCESS: vacuumOn"); }
      break;

    case 'p':  // print current position
      if (!softStopped) { 
        Serial.print("SUCCESS: printCurrentPosXYZ ");
        Serial.print(xPos);
        Serial.print(", ");
        Serial.print(yPos);
        Serial.print(", ");
        Serial.println(zPos);
      }
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
