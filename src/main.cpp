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
void doMotor(char* cmd) { command.motor(&motor, cmd); }

#define ENABLE_CURRENT_SENSING

#ifdef ENABLE_CURRENT_SENSING
InlineCurrentSense current_sense  = InlineCurrentSense((float)0.01, (float)50.0, 26, 27, _NC);
#endif
void setup() {

  // use monitoring with serial 
  Serial.begin(115200);

  delay(3000);
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

#ifdef ENABLE_CURRENT_SENSING
  // link current sense and driver
  current_sense.linkDriver(&driver);
  // init current sense
  if (current_sense.init())
    Serial.println("Current sense init success!");
  else{
    Serial.println("Current sense init failed!");
    return;
  }

  // link motor and current sense
  motor.linkCurrentSense(&current_sense);

  // for SimpleFOCShield v2.01/v2.0.2
  current_sense.gain_b *= -1;
  // skip alignment
  // current_sense.skip_align = true;
#endif
  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 12;   // [V]
 
   // choose FOC modulation (optional)
  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  
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

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

    // align sensor and start FOC
  motor.initFOC();

  // add the motor to the commander interface
  // The letter (here 'M') you will provide to the SimpleFOCStudio
  command.add('M',doMotor,"motor");
  // tell the motor to use the monitoring
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 0; // disable monitor at first - optional

  _delay(1000);

  Serial.print("Current sense gains: a = ");
  Serial.print(current_sense.offset_ia); // milli Amps
  Serial.print("\tb=");
  Serial.print(current_sense.offset_ib); // milli Amps
  Serial.print("\tc=");
  Serial.println(current_sense.offset_ic); // milli Amps
}

// 输入M100 即目标是100，根据motiontype不同，可以是角度，速度，力矩
void loop() {
  motor.loopFOC();

  // this function can be run at much lower frequency than loopFOC()
  motor.move();

#ifdef ENABLE_CURRENT_SENSING

  PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  float current_magnitude = current_sense.getDCCurrent();

  Serial.print(currents.a*1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents.b*1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents.c*1000); // milli Amps
  Serial.print("\t");
  Serial.println(current_magnitude*1000); // milli Amps

#endif

  // real-time monitoring calls
  motor.monitor();
  // user communication
  command.run();

}
