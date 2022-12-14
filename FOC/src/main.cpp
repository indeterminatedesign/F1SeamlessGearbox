#include <SimpleFOC.h>
#include <Wire.h>

// Stepper motor instance
StepperMotor leftMotor = StepperMotor(50);
StepperMotor rightMotor = StepperMotor(50);

#define motorPinA 27
#define motorPinB 14

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 10;
const int resolution = 8;
int dutyCycle = 75;

const uint32_t barrelPhasingDelay = 1e4; // in microseconds

// Stepper driver instance
StepperDriver4PWM leftDriver = StepperDriver4PWM(32, 33, 25, 26);
StepperDriver4PWM rightDriver = StepperDriver4PWM(19, 18, 17, 16);
// sensor instance
MagneticSensorI2C leftSensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C rightSensor = MagneticSensorI2C(AS5600_I2C);

// Left gear position in radians, right gear position in radians
const float_t gearPositions[9][2] =
    {
        {0.8, 1.82},   // Neutral
        {0.23, 1.82},  // 1st
        {-0.34, 2.39}, // 2nd
        {-0.91, 2.96}, // 3rd
        {-1.48, 3.53}, // 4th
        {-2.05, 4.1}, // 5th
        {-2.62, 4.67}, // 6th
        {-3.19, 5.24},  // 7th
        {-3.76, 5.81}, // 8th
};
// 1 means delay, 0 means no delay.  Columns Upshift (L,R)  Columns Downshift (L,R)
const float_t gearBarrelPhasing[9][4] =
    {
        {0, 0, 0, 0}, // Neutral
        {0, 0, 0, 0}, // 1st
        {0, 1, 1, 0}, // 2nd
        {1, 0, 0, 1}, // 3rd
        {0, 1, 1, 0}, // 4th
        {1, 0, 0, 1}, // 5th
        {0, 1, 1, 0}, // 6th
        {1, 0, 0, 1}, // 7th
        {0, 1, 1, 0}, // 8th
};
#define left 0
#define right 1

const float_t leftActuatorStartAngle = 0.8;   // In radians
const float_t rightActuatorStartAngle = 0.28; // In radians

uint32_t dataLogMicros = 0;
const uint32_t dataLogInterval = 1000000;
#define TCAADDR 0x70

void initializeActuator(StepperDriver4PWM &driver, MagneticSensorI2C &sensor, StepperMotor &stepper);

void selectEncoder(uint8_t i)
{
  if (i > 7)
    return;
  //Serial.println("TCA Select Ran");
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup()
{

  Serial.begin(500000);
  // SimpleFOCDebug::enable();

  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Setup Begin");

  // Pull motor pin b low and setup output
  pinMode(motorPinB, OUTPUT);
  pinMode(motorPinA, OUTPUT);

  digitalWrite(motorPinB, LOW);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(motorPinA, pwmChannel);

  // start motor
  ledcWrite(pwmChannel, dutyCycle);

  // initialize encoder sensor hardware
  selectEncoder(left); // Set encoder to appropriate port on TCA9548A
  initializeActuator(leftDriver, leftSensor, leftMotor);

  selectEncoder(right); // Set encoder to appropriate port on TCA9548A
  initializeActuator(rightDriver, rightSensor, rightMotor);

  // align encoder and start FOC
  // 1.55, Direction::CW
  selectEncoder(left);
  leftMotor.initFOC(3.83, Direction::CW);

  selectEncoder(right);
  rightMotor.initFOC(1.43, Direction::CW);

  Serial.println("Motor ready.");
  _delay(1000);
}

float_t shiftIncriment = 0.57;
// angle set point variable
float_t rightTargetAngle = 0;
float_t leftTargetAngle = 0;

// Initialization
bool initializeMotorAngle = true;
uint32_t timestamp_us = micros();

// Throttle blip
bool throttleBlipInProgress = false;
uint32_t timestampShiftBlip = micros();
uint32_t shiftBlipLength = 75e3;

// barrel phasing
uint32_t timestampBarrelPhasing = micros();
uint32_t leftBarrelDelay = 3e4;
uint32_t rightbarrelDelay = 3e4;

int16_t i = 0;
bool incrimentFlag = true;

int32_t startInitialize = micros();

void loop()
{
  if (initializeMotorAngle)
  {
    Serial.print("Left Sensor Start Angle: ");
    Serial.println(leftSensor.getAngle());

    Serial.print("Right Sensor Start Angle: ");
    Serial.println(rightSensor.getAngle());

    leftMotor.P_angle.P = 4;
    rightMotor.P_angle.P = 4;
    leftTargetAngle = gearPositions[0][left];
    rightTargetAngle = gearPositions[0][right];

    Serial.println("Initialize Start");
    if (micros() - startInitialize > 5e6)
    {
      Serial.println("Initialize Complete");
      initializeMotorAngle = false;
      leftMotor.P_angle.P = 35;
      rightMotor.P_angle.P = 35;
    }
  }

  // each one second, shift grears
  if (micros() - timestamp_us > 1e6)
  {

    // Serial.println("Change Angle");
    timestamp_us = micros();
    leftTargetAngle = gearPositions[i][left];
    rightTargetAngle = gearPositions[i][right];
    throttleBlipInProgress = true;
    timestampShiftBlip = micros(); // Start timer for shift motor speed blip
    timestampBarrelPhasing = micros();  //Start timer for shiftbarrel phasing

    if (incrimentFlag)
    {
      i++;
      leftBarrelDelay = gearBarrelPhasing[i][0] * barrelPhasingDelay;
      rightbarrelDelay = gearBarrelPhasing[i][1] * barrelPhasingDelay;
    }
    else
    {
      i--;
      leftBarrelDelay = gearBarrelPhasing[i][2] * barrelPhasingDelay;
      rightbarrelDelay = gearBarrelPhasing[i][3] * barrelPhasingDelay;
    }

    if (i == 8) // 8 gear plus neutral
    {
      incrimentFlag = false;
    }
    if (i == 0)
    {
      incrimentFlag = true;
    }
  }

  if (throttleBlipInProgress)
  {
    ledcWrite(pwmChannel, 0);
    if (micros() - timestampShiftBlip > shiftBlipLength)
    {
      ledcWrite(pwmChannel, dutyCycle);
      throttleBlipInProgress = false;
    }
  }

  selectEncoder(left);

  // main FOC algorithm function
  leftMotor.loopFOC();

  if (micros() - timestampBarrelPhasing > leftBarrelDelay)
  {
    // Motion control function
    leftMotor.move(leftTargetAngle);
  }

  selectEncoder(right);
  // main FOC algorithm function
  rightMotor.loopFOC();

  if(micros() - timestampBarrelPhasing > rightbarrelDelay)
  {
  // Motion control function
  rightMotor.move(rightTargetAngle);
  }
}

void initializeActuator(StepperDriver4PWM &driver, MagneticSensorI2C &sensor, StepperMotor &stepper)
{
  // link the motor to the sensor
  stepper.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  driver.pwm_frequency = 20000;

  driver.init();
  // link driver
  stepper.linkDriver(&driver);

  stepper.controller = MotionControlType::angle;
  stepper.foc_modulation = FOCModulationType::SpaceVectorPWM;

  stepper.voltage_sensor_align = 9;

  // velocity PID controller parameters
  /*  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.001;*/
  // default P=0.5 I = 10 D =0
  stepper.PID_velocity.P = 0.2;
  stepper.PID_velocity.I = 25;
  stepper.PID_velocity.D = 0.001;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  stepper.PID_velocity.output_ramp = 3000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  stepper.LPF_velocity.Tf = 0.01;

  // setting the limits
  // either voltage
  stepper.voltage_limit = 24; // Volts - default driver.voltage_limit
  // of current
  stepper.current_limit = 5; // Amps - default 0.2Amps

  // angle P controller -  default P=20
  stepper.P_angle.P = 35;
  //stepper.P_angle.I = .0001;
  stepper.P_angle.D = 0.3;
  stepper.velocity_limit = 1000;
  // comment out if not needed
  stepper.useMonitoring(Serial);

  // initialize motor
  stepper.init();
}