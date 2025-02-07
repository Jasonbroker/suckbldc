// Open loop motor control example
#include <SimpleFOC.h>
#include <encoders/mt6701/MagneticSensorMT6701SSI.h>
#include <SimpleFOCDrivers.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 3, 4, 5);

// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(PIN_MT_CSN);
// arduino::MbedSPI mySPI(PIN_MT_DATA, PIN_MT_MOSI, PIN_MT_CLOCK);

//target variable
float target_velocity = 0;
// angle set point variable
float target_angle = 0;

// instantiate the commander
Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

    // initialise magnetic sensor hardware
  sensor.init();
  // sensor.init(&mySPI);

  // return;
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 5;
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 12;   // [V]
 
   // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  
  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

    // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 6;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 10;
  // maximal velocity of the position control
  motor.velocity_limit = 20;
  
  // comment out if not needed
  motor.useMonitoring(Serial);

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

    // align sensor and start FOC
  motor.initFOC();

  // add target command T
  // command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");
  command.add('T', doTarget, "target angle");

  Serial.println("Motor ready!");
  Serial.println("Set target angle [rad/s]");


  _delay(1000);
}

void loop() {

  // delay(5);
  // return;
    // display the angle and the angular velocity to the terminal
  // Serial.println(sensor.getSensorAngle());
  // Serial.print("\t");
  // Serial.println(sensor.getVelocity());
  // return;

  motor.loopFOC();
  
  motor.move(target_angle);

  // user communication
  command.run();


}
