#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <SimpleFOC.h>

// LSM9DS1 Library Init
LSM9DS1 imu;

// Example I2C Setup
// SDO_XM and SDO_G are both pulled high, so our addresses are:
// #define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
// #define LSM9DS1_AG 0x6B // Would be 0x6A if SDO_AG is LOW

// BLDC motor & driver instances
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(5, 10, 6, 8);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(3, 9, 11, 7);

float target_velocity1 = 10;
float target_velocity2 = 10;

// PID coefficients
float Kp = 25.0;
float Ki = 0.1;
float Kd = 0.0;

// PID control variables
float prevError = 0.0;
float integral = 0.0;

// Low-pass filter parameters
float alpha = 0.1; // Smoothing factor (0 < alpha <= 1)
float filteredAccel = 0.0;

Commander command = Commander(Serial);
void doTarget1(char* cmd) { command.scalar(&target_velocity1, cmd); }
void doLimit1(char* cmd) { command.scalar(&motor1.voltage_limit, cmd); }
void doTarget2(char* cmd) { command.scalar(&target_velocity2, cmd); }
void doLimit2(char* cmd) { command.scalar(&motor2.voltage_limit, cmd); }

// Sketch Output Settings
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Function definitions
void printAccel();

void setup()
{
  driver1.voltage_power_supply = 12;
  driver2.voltage_power_supply = 12;

  driver1.voltage_limit = 6;
  driver1.init();
  driver2.voltage_limit = 6;
  driver2.init();

  // Link the motors and the drivers
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  // Limit the voltage to be set to the motors
  motor1.voltage_limit = 3; // [V]
  motor2.voltage_limit = 3;

  // Open loop control config
  motor1.controller = MotionControlType::velocity_openloop;
  motor2.controller = MotionControlType::velocity_openloop;

  // Init motor hardware
  motor1.init();
  motor2.init();

  // Add target command T
  command.add('T', doTarget1, "target velocity");
  command.add('L', doLimit1, "voltage limit");
  command.add('T', doTarget2, "target velocity");
  command.add('L', doLimit2, "voltage limit");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  Wire.begin();

  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will work for an out of the box LSM9DS1 Breakout, but may need to be modified if the board jumpers are.");
    while (1);
  }
}

void loop()
{
  // Update the sensor values whenever new data is available
  if (imu.accelAvailable())
  {
    // Read from the accelerometer
    imu.readAccel();
  }

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printAccel(); // Print "A: ax, ay, az"
    Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }
  
  // Apply low-pass filter to the accelerometer data
  filteredAccel = alpha * imu.calcAccel(imu.ax) + (1 - alpha) * filteredAccel;
  
  // Calculate the steady-state error
  float error = filteredAccel;
  
  // PID calculations
  integral += error;
  float derivative = error - prevError;
  float output = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;
  
  // Adjust the target velocity based on the PID output
  float adjustedVelocity1 = output;
  float adjustedVelocity2 = -output;

  // Set the motor velocity
  motor1.move(adjustedVelocity1);
  motor2.move(adjustedVelocity2);

  command.run();
}

void printAccel()
{
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif
}
