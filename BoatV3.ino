#include <Wire.h>
#include <Servo.h>
//#include <PID_v1.h>
#include <MPU6050.h>

Servo escLeft;
Servo escRight;

MPU6050 mpu;

const int motorLeftPin = 9;
const int motorRightPin = 10;

// Time keeper
unsigned long lastTime_G = 0;
unsigned long lastTime_PID = 0;

// Z rotation
double yaw = 0;

// Gyroscope filtering parameter
double gyroBias = 0;

// PID Variables
// Desire Heading
double setpoint;

// Boat parameters
double baseSpeed = 1155;

// PID parameters
double Kp = 1.2, Ki = 0.02, Kd = 1.0;

// PID Variables
double prevError = 0;
double integral = 0;

// Path tracking
int currentLeg = 0;
unsigned long legStartTime;

// Leg Timing
unsigned long leg1Time = 25000;
unsigned long turnTime = 25000;
unsigned long leg3Time = 40000;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 initialized successfully.");


  // Attach ESCs
  escLeft.attach(motorLeftPin);
  escRight.attach(motorRightPin);

  // Start with neutral throttle
  escLeft.writeMicroseconds(0);
  escRight.writeMicroseconds(0);

  delay(5000);

  calibrateGyro();

  setpoint = 0;
  delay(5000);

  legStartTime = millis();
}

void loop() {

  unsigned long currentTime = millis();
  unsigned long dt = currentTime - legStartTime;


  switch (currentLeg) {
    case 0:
      // Leg 1: Go straight at heading = 0 for about 40 seconds
      setpoint = 30;
      if (dt > leg1Time) {
        legStartTime = currentTime;
        currentLeg++;
        integral = 0;
        prevError = 0;
      }
      break;
    case 1:
      // Leg 2: Curve Left
      setpoint = 90;
      if (dt > turnTime) {
        legStartTime = currentTime;
        currentLeg++;
        integral = 0;
        prevError = 0;
      }
      break;
    case 2:
      // Leg 3: Aim Home
      setpoint = 170;
      if (dt > leg3Time) {
        legStartTime = currentTime;
        currentLeg++;
      }
      break;
  }

  double current = readHeading();
  double correction = 1.15 * computePID(setpoint, current);

  double LMS = baseSpeed - correction;
  double RMS = baseSpeed + correction;

  LMS = constrain(LMS, 1000, 2000);
  RMS = constrain(RMS, 1000, 2000);

  escLeft.writeMicroseconds(LMS);
  escRight.writeMicroseconds(RMS);

  delay(50);
}

// Read heading from gyroscope
double readHeading() {
  unsigned long currentTime_G = millis();
  // Time in seconds
  float dt = (currentTime_G - lastTime_G) / 1000.0;
  lastTime_G = currentTime_G;

  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // Covert gyro raw value (degrees per second)
  float gyroYawRate = (gz - gyroBias) / 131.0;
  // Integrate to get yaw angle
  yaw += gyroYawRate * dt;
  // Apply drift correction
  //yaw *= alpha;

  // Apply clamping
  if (yaw < 0) yaw += 360;
  if (yaw > 360) yaw -= 360;

  return yaw;
}

// Calibrate Gyroscope Offset
void calibrateGyro() {
    Serial.println("Calibrating gyroscope... Keep the sensor still!");
    long sum = 0;
    int samples = 500;

    for (int i = 0; i < samples; i++) {
        int16_t gx, gy, gz;
        mpu.getRotation(&gx, &gy, &gz);
        sum += gz;
        delay(5);
    }
    
    gyroBias = sum / (float)samples;  // Get average bias
    Serial.print("Gyro bias: ");
    Serial.println(gyroBias);
}

// Returns correction data
double computePID(double target, double current)
{
  // Get elapsed time
  unsigned long currentTime_PID = millis();
  double dt = (currentTime_PID - lastTime_PID) / 1000.0;
  lastTime_PID = currentTime_PID;

  // Compute shortest path error (-180 to 180)
  double error = target - current;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  // Integral term (sum of errors)
  integral += error * dt;

  // Derivative term (rate of change of error)
  double derivative = (error - prevError) / dt;
  prevError = error;

  // Compute PID output
  double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  return output;
}

