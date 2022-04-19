/*  By Kevi Aday

Flight controller program for communication with HC-12 module
and control over a quadcopter drone using PID stabilization.

*/

#include <Servo.h>
#include <SoftwareSerial.h>
#include <PID_v2.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
//#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

#define HC12_SET_PIN 8
#define HC12_TX_PIN 6
#define HC12_RX_PIN 7
#define SERVO_PIN_START 9

#define NUM_MOTORS 4
#define MIN_PULSE_WIDTH_ 1000
#define MAX_PULSE_WIDTH_ 2000
#define MIN_SERVO_VALUE 0
#define MAX_SERVO_VALUE 180

#define USB_BAUD_RATE 9600
#define BAUD_RATE 9600 // 115200;

#define MIN_THRUST_OUTPUT 0
#define MIN_PARAM_VAL -100
#define MAX_PARAM_VAL 100
#define MAX_ANGLE 35.0    // Max angle at which the drone can tilt for orientation in degrees
#define MAX_VELOCITY 8.0  // Max vertical velocity in m/s
#define MAX_ANGULAR_VELOCITY 60.0 // Maximum velocity for yawing in degree/s
//#define GRAVITY 9.885      // Acceleration due to gravity constant

#define MAX_MESSAGE_LENGTH 9
#define NUM_MEASUREMENTS 30 // Amount of calibration measurements to take
#define CALIB_SLEEP 150   // How much delay between measurements for calibration (ms)

/* Remote control constants */
#define COMMAND 'C'
#define COMMAND_END '\n'
#define COMMAND_MOVE 'M'
#define COMMAND_AT "AT"
#define MOVE "CM"
#define COMMAND_ALL_STOP 'S'
#define COMMAND_CALIBRATE 'G'

/* Constants for offsetting the measured values for manual calibration */
#define COMMAND_OFFSET_VERTICAL 'V'
#define COMMAND_OFFSET_YAW 'Y'
#define COMMAND_OFFSET_PITCH 'P'
#define COMMAND_OFFSET_ROLL 'R'
#define COMMAND_OFFSET_UP 'U'
#define COMMAND_OFFSET_DOWN 'D'
#define COMMAND_OFFSET_RESET 'X'

/* How much to increment/decrement when manually calibrating */
#define OFFSET_VERTICAL_INCREMENT 0.2
#define OFFSET_YAW_INCREMENT 1.
#define OFFSET_PITCH_INCREMENT 1.
#define OFFSET_ROLL_INCREMENT 1.

#define LOGGING_INTERVAL 1000 // How often to log debug info to Serial in ms

#define PID_UPDATE_INTERVAL 50  // Update interval for PID calculations and sensor measurements in ms
#define DATA_SMOOTHING_INTERVAL 50 // Interval in ms between the averaging of sensor data to eliminate instantaneous spikes/random fluctuations
//#define PID_UPDATE_FREQUENCY = 1 / PID_UPDATE_INTERVAL;  // Frequency based on update interval (units = s^-1)
//#define MOTOR_UPDATE_INTERVAL 100

/* PID constants */
double Kp[4] = {0.7, 0.7, 0.05, 4.0};     // P coefficients in the order: Pitch, Roll, Yaw, Thrust
double Ki[4] = {0.2, 0.2, 0.1, 8.0};     // I coefficients in the order: ...
double Kd[4] = {0.1, 0.1, 0, 2.7};     // D coefficients in the order: ...

SoftwareSerial HC12(HC12_TX_PIN, HC12_RX_PIN);
// SoftwareSerial MainDevice(0, 1);

/* Motor values in arrays always start with motor 1 and end with the last motor, in order */
Servo motors[NUM_MOTORS];
int currentSpeeds[NUM_MOTORS];  // Stores the current speed of each motor
int totalThrust = MIN_SERVO_VALUE;  // The total amount of thrust (sum of all motor speeds)
bool allStop = true;

char _incomingByte;
char readBuffer[MAX_MESSAGE_LENGTH];
bool msgEnd = false;
int bufferIndex;

int smoothingCounter = 0;
bool firstMeasure = true;

int currentThrust = 0;  /* Range: [0, 100] */
int currentPitch = 0;   /* Range of the rest: [-100, 100] */
int currentRoll = 0;
int currentYaw = 0;

/* PID variables (desired, measured, and output values respectively) */
double pitchAngle;
double pitchAngleInput = 0;
double pitchMeasured;
double pitchSmoothing = 0;
double pitchOutput;
float pitchOffset = 0;

double rollAngle;
double rollAngleInput = 0;
double rollMeasured;
double rollSmoothing = 0;
double rollOutput;
float rollOffset = 0;

double lastHeading;
double headingVelocity;
double headingVelocityInput = 0;
double headingVelocityMeasured;
double headingMeasured;
double headingSmoothing = 0;
double yawOutput;
float yawOffset = 0;

double gravityMeasured = 0;
double verticalVelocity;
double verticalVelocityInput = 0;
double verticalVelocityMeasured;
double lastVerticalVelocity;
double verticalAccelMeasured;
double verticalAccelSmoothing = 0;
double thrustOutput;
float verticalOffset = 0;

double lastMeasureTime = 0; // Last time that measurements were made

PID pitchPID(&pitchAngleInput, &pitchOutput, &pitchAngle, Kp[0], Ki[0], Kd[0], DIRECT);
PID rollPID(&rollAngleInput, &rollOutput, &rollAngle, Kp[1], Ki[1], Kd[1], DIRECT);
//PID yawPID(&headingVelocityMeasured, &yawOutput, &headingVelocity, Kp[2], Ki[2], Kd[2], DIRECT);
PID thrustPID(&verticalVelocityInput, &thrustOutput, &verticalVelocity, Kp[3], Ki[3], Kd[3], DIRECT);

float pidUpdateTime = 0;
float logUpdateTime = 0;
float motorUpdateTime = 0;
float dataSmoothTime;
float pidLoopTime;

/* Sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
//Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

//float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;


void initSensors() {
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }

  /*
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections
    Serial.println(F("Ooops, no BMP180 detected ... Check your wiring!"));
    while(1);
  }
  */
}

void setTotalThrust(int spd, int delayMs = 0) {
  calcMotorSpeeds(spd, 0, 0, 0);
  updateMotors();
  if (delayMs) delay(delayMs);
}

void debugArray(int arr[], int len, String sep = "\t") {
  for (int i = 0; i < len; i++)
    Serial.print(String(arr[i]) + sep);
  Serial.println();
}

float gravityMagnitude(sensors_event_t accelEvent) {
  return sqrtf(powf(accelEvent.acceleration.x, 2) + powf(accelEvent.acceleration.y, 2) + powf(accelEvent.acceleration.z, 2));
}

float measureGravity() {
  float gravity = 0;
  sensors_event_t event;

  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
    accel.getEvent(&event);
    gravity += gravityMagnitude(event);
    delay(CALIB_SLEEP);
  }
  gravity /= NUM_MEASUREMENTS;

  return gravity;
}

void calibrate() {
  Serial.println(F("Calibrating drone..."));
  unsigned long start = millis();

  //gravityMeasured = measureGravity();
  firstMeasure = true;
  _resetSmoothing();
  resetPID();
  pitchMeasured = 0;
  rollMeasured = 0;
  headingVelocityMeasured = 0;
  lastHeading = 0;
  verticalVelocityMeasured = 0;
  lastVerticalVelocity = 0;
  takeMeasurements();

  Serial.print(F("Calibration complete in ")); Serial.print(millis()-start); Serial.println(F(" ms."));
  Serial.print(F("Measured gravity: ")); Serial.println(gravityMeasured);
}

void resetOffsets() {
  pitchOffset = 0;
  rollOffset = 0;
  yawOffset = 0;
  verticalOffset = 0;
}


void setup() {
  pinMode(HC12_SET_PIN, OUTPUT);
  digitalWrite(HC12_SET_PIN, HIGH);

  // MainDevice.begin(BAUD_RATE);
  Serial.begin(USB_BAUD_RATE);
  HC12.begin(BAUD_RATE);

  for (int i = 0; i < NUM_MOTORS; i++) {
    attachServo(motors[i], SERVO_PIN_START+i);
  }
  delay(250);
  allMotors(MAX_PULSE_WIDTH_);
  delay(1500);
  allMotors(MIN_PULSE_WIDTH_);
  Serial.println(F("Motors armed."));
  delay(200);

  initSensors();
  calibrate();

  thrustPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  //yawPID.SetMode(AUTOMATIC);
  thrustPID.SetOutputLimits(MIN_THRUST_OUTPUT, MAX_PARAM_VAL);
  pitchPID.SetOutputLimits(MIN_PARAM_VAL, MAX_PARAM_VAL);
  rollPID.SetOutputLimits(MIN_PARAM_VAL, MAX_PARAM_VAL);
  //yawPID.SetOutputLimits(MIN_PARAM_VAL, MAX_PARAM_VAL);
  thrustPID.SetSampleTime(PID_UPDATE_INTERVAL);
  pitchPID.SetSampleTime(PID_UPDATE_INTERVAL);
  rollPID.SetSampleTime(PID_UPDATE_INTERVAL);
  //yawPID.SetSampleTime(PID_UPDATE_INTERVAL);

  commandHC12("AT+B" + String(BAUD_RATE));
  
  /* Send the PID constants to the CSV logfile */
  HC12.print(F("PID Constants (Kp Ki Kd),"));
  for (int i = 0; i < 4; i++) {
    HC12.print(String(Kp[i]) + " " + String(Ki[i]) + " " + String(Kd[i]));
    if (i < 3) HC12.print(F(","));
  }
  HC12.println();
  /* Data header names */
  HC12.println(F("m. vertical velocity,m. pitch angle,m. roll angle,m. angular velocity,d. vertical velocity,d. pitch angle,d. roll angle,d. angular velocity,raw thrust,raw pitch,raw roll,raw yaw,pid thrust,pid pitch,pid roll,pid yaw"));

  delay(2000);

  dataSmoothTime = millis() + DATA_SMOOTHING_INTERVAL;
}

void loop() {
  bufferIndex = 0;
  msgEnd = false;
  
  while (HC12.available() && bufferIndex < MAX_MESSAGE_LENGTH) {  // While data is available in HC12
    _incomingByte = HC12.read();  // Read a byte from the HC12
    if (_incomingByte == COMMAND_END) {
      msgEnd = true;
      break;
    }
    readBuffer[bufferIndex] = _incomingByte;  // Add byte to the data buffer
    bufferIndex++;
    delay(2);
  }
  
  if (msgEnd) {
    //Serial.println("Got message: " + String(readBuffer));
    if (readBuffer[0] == COMMAND) execCommand();
  }

  if (allStop) return;

  calcDesiredVars();
  calcMeasuredVars();
  updatePID();
  updateMotors();

  /*
  if (logUpdateTime < millis()) {
    
    HC12.print(String(verticalVelocityMeasured) + "," + String(pitchMeasured) + "," + String(rollMeasured) + "," + String(headingVelocityMeasured) + ",");
    HC12.print(String(verticalVelocity) + "," + String(pitchAngle) + "," + String(rollAngle) + "," + String(headingVelocity) + ",");
    HC12.print(String(currentThrust) + "," + String(currentPitch) + "," + String(currentRoll) + "," + String(currentYaw) + ",");
    HC12.println(String(thrustOutput) + "," + String(pitchOutput) + "," + String(rollOutput) + "," + String(yawOutput));

    Serial.print(F("Raw thrust: ")); Serial.println(currentThrust);
    Serial.println("Raw pitch: " + String(currentPitch));
    Serial.println("Raw roll: " + String(currentRoll));
    Serial.println("Raw yaw: " + String(currentYaw));
    int thrust = currentThrust >= 0. ? currentThrust : 0.;
    calcMotorSpeeds(thrust, currentPitch, currentRoll, currentYaw);
    Serial.print(F("Raw motor speeds: "));
    debugArray(currentSpeeds, NUM_MOTORS);

    Serial.println("d. vertical velocity: " + String(verticalVelocity));
    Serial.println("d. pitch angle: " + String(pitchAngle));
    Serial.println("d. roll angle: " + String(rollAngle));
    Serial.println("d. yaw velocity: " + String(headingVelocity));

    Serial.println("m. vertical velocity: " + String(verticalVelocityMeasured));
    Serial.println("m. pitch angle: " + String(pitchMeasured));
    Serial.println("m. roll angle: " + String(rollMeasured));
    Serial.println("m. yaw velocity: " + String(headingVelocityMeasured));
    Serial.print(F("m. heading: ")); Serial.println(headingMeasured);

    Serial.print(F("PID thrust: ")); Serial.println(thrustOutput);
    Serial.print(F("PID pitch: ")); Serial.println(pitchOutput);
    Serial.print(F("PID roll: ")); Serial.println(rollOutput);
    Serial.print(F("PID yaw: ")); Serial.println(yawOutput);
    Serial.print(F("PID motor speeds: "));
    calcMotorSpeeds(thrustOutput, pitchOutput, rollOutput, yawOutput);
    debugArray(currentSpeeds, NUM_MOTORS);

    Serial.print(F("PID/measurement loop time: ")); Serial.println(pidLoopTime);
    Serial.println();

    logUpdateTime = millis() + LOGGING_INTERVAL;
  }
  */
}

void test() {
  delay(5000);
  setTotalThrust(15, 2000);
  setTotalThrust(10, 2000);
  setTotalThrust(5, 2000);
  setTotalThrust(0);
}

/*
float getHeight() {
	sensors_event_t bmp_event;

	// Calculate the altitude using the barometric pressure sensor
	bmp.getEvent(&bmp_event);

	// Get ambient temperature in C
	float temperature;
	bmp.getTemperature(&temperature);

	// Convert atmospheric pressure, SLP and temp to height
	return bmp.pressureToAltitude(seaLevelPressure,
							      bmp_event.pressure,
							      temperature);
}
*/

bool getOrientation(sensors_vec_t *orientation) {
	sensors_event_t accel_event;
	sensors_event_t mag_event;

	/* Read the accelerometer and magnetometer */
	accel.getEvent(&accel_event);
	mag.getEvent(&mag_event);

	return dof.fusionGetOrientation(&accel_event, &mag_event, orientation);
}

int byteToInt(unsigned char c) {
  return (int)c - 100;
}

void execCommand() {
  char command = readBuffer[1];
  char subCommand = readBuffer[2];

  if (command == COMMAND_MOVE) {
    currentThrust = byteToInt(readBuffer[2]);
    currentPitch = byteToInt(readBuffer[4]);
    currentRoll = byteToInt(readBuffer[3]);
    currentYaw = byteToInt(readBuffer[5]);

    //Serial.println(String(currentThrust) + '\t' + String(currentPitch) + '\t' + String(currentRoll) + '\t' + String(currentYaw));
  }
  else if(command == COMMAND_AT[0]) {
    commandHC12("AT+" + String(readBuffer));
  }
  else if(command == COMMAND_ALL_STOP) {
    if (!allStop) {
      Serial.println(F("All stop received! Stopping motors."));
      emergyencyStop();
    }
    else {
      Serial.println(F("Starting motors."));
      allStop = false;
    }
  }
  else if (command == COMMAND_CALIBRATE)
    calibrate();
  
  /* Manual calibration commands */
  else if (command == COMMAND_OFFSET_VERTICAL) {
    Serial.println(F("Applying vertical offset."));
    if (subCommand == COMMAND_OFFSET_UP) verticalOffset += OFFSET_VERTICAL_INCREMENT;
    else if (subCommand == COMMAND_OFFSET_DOWN) verticalOffset -= OFFSET_VERTICAL_INCREMENT;
  }
  else if (command == COMMAND_OFFSET_PITCH) {
    Serial.println(F("Applying pitch offset."));
    if (subCommand == COMMAND_OFFSET_UP) pitchOffset += OFFSET_PITCH_INCREMENT;
    else if (subCommand == COMMAND_OFFSET_DOWN) pitchOffset -= OFFSET_PITCH_INCREMENT;
  }
  else if (command == COMMAND_OFFSET_ROLL) {
    Serial.println(F("Applying roll offset."));
    if (subCommand == COMMAND_OFFSET_UP) rollOffset += OFFSET_ROLL_INCREMENT;
    else if (subCommand == COMMAND_OFFSET_DOWN) rollOffset -= OFFSET_ROLL_INCREMENT;
  }
  else if (command == COMMAND_OFFSET_YAW) {
    Serial.println(F("Applying yaw offset."));
    if (subCommand == COMMAND_OFFSET_UP) yawOffset += OFFSET_YAW_INCREMENT;
    else if (subCommand == COMMAND_OFFSET_DOWN) yawOffset -= OFFSET_YAW_INCREMENT;
  }
  else if (command == COMMAND_OFFSET_RESET) {
    Serial.println(F("Offsets reset."));
    resetOffsets();
  }
}

void emergyencyStop() {
  allMotors(MIN_PULSE_WIDTH_);
  allStop = true;
  currentThrust = 0;
  currentPitch = 0;
  currentRoll = 0;
  currentYaw = 0;
  verticalVelocity = 0;
  pitchAngle = 0;
  rollAngle = 0;
  headingVelocity = 0;
  thrustOutput = 0;
  pitchOutput = 0;
  rollOutput = 0;
  yawOutput = 0;
}

void calcDesiredVars() {
  /* Convert thrust range [0, 100] to desired velocity in m/s */
  verticalVelocity = (currentThrust / 100.) * MAX_VELOCITY;

  /* Convert orientation range [-100, 100] to desired angles in degrees */
  pitchAngle       = (currentPitch / 100.)  * MAX_ANGLE;
  rollAngle        = (currentRoll / 100.)   * MAX_ANGLE;
  headingVelocity  = (currentYaw / 100.)    * MAX_ANGULAR_VELOCITY;

  /*
  Serial.println("d. vertical velocity: " + String(verticalVelocity));
  Serial.println("d. pitch angle: " + String(pitchAngle));
  Serial.println("d. roll angle: " + String(rollAngle));
  Serial.println("d. yaw velocity: " + String(headingVelocity));
  */
}

bool rawSensorData(double &pitch, double &roll, double &heading, double &verticalAccel) {
  sensors_vec_t orientation;
  sensors_event_t accelEvent;

  if (getOrientation(&orientation) && accel.getEvent(&accelEvent)) {
    pitch = orientation.pitch;
    roll = orientation.roll;
    heading = orientation.heading;
    verticalAccel = accelEvent.acceleration.z;

    return true;
  }

  return false;
}

void _updateSmoothing(double pitch, double roll, double yaw, double accel) {
  pitchSmoothing += pitch;
  rollSmoothing += roll;
  headingSmoothing += yaw;
  verticalAccelSmoothing += accel;
  smoothingCounter++;
}

void _resetSmoothing() {
  smoothingCounter = 0;
  pitchSmoothing = 0;
  rollSmoothing = 0;
  headingSmoothing = 0;
  verticalAccelSmoothing = 0;

  dataSmoothTime = millis() + DATA_SMOOTHING_INTERVAL;
}

bool takeMeasurements() {
  double tempPitch;
  double tempRoll;
  double tempYaw;
  double tempAccel;

  /* It's time to smooth the data by averaging accumulated values and reset the smoothing counter
   * and the next smoothing time. Do not smooth if it's the first measurement or if there's no data yet. */
  if (dataSmoothTime < millis() && smoothingCounter != 0 && !firstMeasure) {
    pitchMeasured = pitchSmoothing / smoothingCounter;
    rollMeasured = rollSmoothing / smoothingCounter;
    headingMeasured = headingSmoothing / smoothingCounter;
    verticalAccelMeasured = verticalAccelSmoothing / smoothingCounter;

    _resetSmoothing();
  }

  /* Update smoothing, even if smoothing was just reset. */
  if (rawSensorData(tempPitch, tempRoll, tempYaw, tempAccel)) {
    _updateSmoothing(tempPitch, tempRoll, tempYaw, tempAccel);
    
    if (firstMeasure) {
      pitchMeasured = tempPitch;
      rollMeasured = tempRoll;
      headingMeasured = lastHeading = tempYaw;
      verticalAccelMeasured = tempAccel;

      firstMeasure = false;
    }

    return true;
  }

  return false;
}

void calcMeasuredVars() {
  double measureTime = millis();
  double deltaTime = (measureTime - lastMeasureTime) / 1000.;

  /* Set the measured values of the drone's orientation in space, also applying smoothing to data */
  takeMeasurements();

  /* Calculate measured vertical velocity using numerical integration (v = v0 + a*t, where v0 is always the last velocity and t is the change in time).
   * The initial vertical acceleration is subtracted from acceleration to cancel the measured acceleration due to gravity.
   * Subtraction is used because the measured vertical acceleration is inverted. */
  verticalVelocityMeasured = lastVerticalVelocity - (verticalAccelMeasured - gravityMeasured * cos(rollMeasured*DEG_TO_RAD) * cos(pitchMeasured*DEG_TO_RAD)) * deltaTime;
  //verticalVelocityMeasured += verticalOffset; // Apply offset specified by user

  /* Calculate the measured rotational velocity (delta heading / delta time) in degree/s */
  //headingVelocityMeasured = (headingMeasured - lastHeading) / deltaTime;
  //headingVelocityMeasured += yawOffset;

  /* Also apply offset to pitch and roll, after the vertical velocity calculation.
   * It must be done after to avoid having a significant influence on vertical stabilization.
   * Does not accumulate because measurements are set right before each calculation. */
  /*
  pitchMeasured += pitchOffset;
  rollMeasured += rollOffset;
  */

  //Serial.print(F("Acceleration: ")); Serial.println(String(event.acceleration.x) + String(event.acceleration.y) + String(event.acceleration.z));

  /*
  Serial.println("m. vertical velocity: " + String(verticalVelocityMeasured));
  Serial.println("m. pitch angle: " + String(pitchMeasured));
  Serial.println("m. roll angle: " + String(rollMeasured));
  Serial.println("m. yaw velocity: " + String(headingVelocityMeasured));
  */

  /* Set the new previous velocity, heading, and time (for derivative and integral calculations) */
  lastVerticalVelocity = verticalVelocityMeasured;
  //lastHeading = headingMeasured;
  lastMeasureTime = measureTime;
}

void resetPID() {
  thrustOutput = 0;
  verticalVelocity = 0;
  verticalVelocityMeasured = 0;
  pitchOutput = 0;
  pitchAngle = 0;
  pitchMeasured = 0;
  rollOutput = 0;
  rollAngle = 0;
  rollMeasured = 0;
  yawOutput = 0;
  headingVelocity = 0;
  headingVelocityMeasured = 0;

  thrustPID.SetMode(MANUAL);
  thrustPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(MANUAL);
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(MANUAL);
  rollPID.SetMode(AUTOMATIC);
  //yawPID.SetMode(MANUAL);
  //yawPID.SetMode(AUTOMATIC);
}

void updatePID() {
  /* Print the raw motor speeds before stabilization
   * for debugging */
  /*
  Serial.print(F("Raw thrust: ")); Serial.println(currentThrust);
  Serial.println("Raw pitch: " + String(currentPitch));
  Serial.println("Raw roll: " + String(currentRoll));
  Serial.println("Raw yaw: " + String(currentYaw));
  int thrust = currentThrust >= 0. ? currentThrust : 0.;
  calcMotorSpeeds(thrust, currentPitch, currentRoll, currentYaw);
	Serial.print(F("Raw motor speeds: "));
	debugArray(currentSpeeds, NUM_MOTORS);
  */

  verticalVelocityInput = verticalVelocityMeasured + verticalOffset;
  pitchAngleInput = pitchMeasured + pitchOffset;
  rollAngleInput = rollMeasured + rollOffset;
  headingVelocityInput = headingVelocityMeasured + yawOffset;

  /* Run the PID for each respective flight variable
   * based on measured and desired values. The output
   * variables determine the adjusted values for
   * stabilization. */
  //thrustPID.Compute();
  thrustOutput = (currentThrust + 100) / 2;
  pitchPID.Compute();
  rollPID.Compute();
  yawOutput = currentYaw;
  //yawPID.Compute();

  /* Make sure the output values are in the right range */
  /*
  thrustOutput = thrustOutput < MIN_THRUST_OUTPUT ? MIN_THRUST_OUTPUT : thrustOutput;
  thrustOutput = thrustOutput > MAX_PARAM_VAL ? MAX_PARAM_VAL : thrustOutput;
  pitchOutput = pitchOutput < MIN_PARAM_VAL ? MIN_PARAM_VAL : pitchOutput;
  pitchOutput = pitchOutput > MAX_PARAM_VAL ? MAX_PARAM_VAL : pitchOutput;
  rollOutput = rollOutput < MIN_PARAM_VAL ? MIN_PARAM_VAL : rollOutput;
  rollOutput = rollOutput > MAX_PARAM_VAL ? MAX_PARAM_VAL : rollOutput;
  yawOutput = yawOutput < MIN_PARAM_VAL ? MIN_PARAM_VAL : yawOutput;
  yawOutput = yawOutput > MAX_PARAM_VAL ? MAX_PARAM_VAL : yawOutput;
  */

  /* Then mix the values together to calculate
   * a speed value for each motor */
  /*
  Serial.print(F("PID thrust: ")); Serial.println(thrustOutput);
  Serial.print(F("PID pitch: ")); Serial.println(pitchOutput);
  Serial.print(F("PID roll: ")); Serial.println(rollOutput);
  Serial.print(F("PID yaw: ")); Serial.println(yawOutput);
  */
	calcMotorSpeeds(thrustOutput, pitchOutput, rollOutput, yawOutput);
  /*
	Serial.print(F("PID motor speeds: "));
	debugArray(currentSpeeds, NUM_MOTORS);
  */
}

void attachServo(Servo servo, int pin) {
  servo.attach(pin, MIN_PULSE_WIDTH_, MAX_PULSE_WIDTH_);
}

void writeServo(int servoIndex, int microSeconds) {
  motors[servoIndex].writeMicroseconds(microSeconds);
  //currentSpeeds[servoIndex] = microSeconds;
}

void commandHC12(String command) {
  Serial.print(F("Executing AT command: ")); Serial.println(command);
  digitalWrite(HC12_SET_PIN, LOW);
  delay(40);
  HC12.print(command);
  delay(40);
  Serial.print(F("Command response: "));
  while (HC12.available())             // If HC-12 has data (the AT command OK response)
    Serial.write(HC12.read());         // Send the data to Serial monitor
  digitalWrite(HC12_SET_PIN, HIGH);
  delay(80);
}

/* Directly write a PWM value to all motors */
void allMotors(int microSeconds) {
  for(int i = 0; i < NUM_MOTORS; i++) {
    writeServo(i, microSeconds);
  }
}

int _setThrustRange(int param, int thrust) {
  return map(param, MIN_PARAM_VAL, MAX_PARAM_VAL, -thrust, thrust);
}

/*
  Calculate the speed of each motor based on the 4 values.
  Creates a linear matrix system (4 variable, 4 equations)
  and when solved, finds the independent value of each motor.
  Thrust should be in the range [0, 100] while all others
  should be [-100, 100].
*/
void calcMotorSpeeds(int thrust, int pitch, int roll, int yaw) {
  pitch = _setThrustRange(pitch, thrust);
  roll = _setThrustRange(roll, thrust);
  yaw = _setThrustRange(yaw, thrust);
  thrust = 4 * thrust - (abs(pitch) + abs(roll) + abs(yaw));
  currentSpeeds[0] = thrust - pitch + roll + yaw;
  currentSpeeds[1] = thrust - pitch - roll - yaw;
  currentSpeeds[2] = thrust + pitch - roll + yaw;
  currentSpeeds[3] = thrust + pitch + roll - yaw;

  for (int i = 0; i < NUM_MOTORS; i++) {
    currentSpeeds[i] = map(currentSpeeds[i] / 4, 0, MAX_PARAM_VAL, MIN_PULSE_WIDTH_, MAX_PULSE_WIDTH_);
  }
}

/*
  Apply calculated speeds to each respective motor
  from currentSpeeds array.
*/
void updateMotors() {
  for(int i = 0; i < NUM_MOTORS; i++) {
    writeServo(i, currentSpeeds[i]);
  }
}
