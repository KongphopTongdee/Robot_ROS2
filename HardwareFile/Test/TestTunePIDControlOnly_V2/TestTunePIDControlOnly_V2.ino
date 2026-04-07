// ---------- Import library ----------

// Import library standard C input/output library.
#include <stdio.h>

// Import the library for reading the encoder position
#include <ESP32Encoder.h>

// Import the core API function( pinMode(), digitalWrite(), etc ), constants and macros( HIGH, LOW, min(), max() )
#include <Arduino.h>

// Import the library for using PID control
#include <PID_v2.h>

// Import the library for using mathematic
#include <math.h>

// ---------- Declare pin usage ----------

// Motor 1 pin usage
#define PWMpinMotorLeft 25     // PWM pin
#define DIRpinMotorLeft 26     // DIR pin
#define encoderD3MotorLeft 32  // Encoder D3 pin
#define encoderD2MotorLeft 33  // Encoder D2 pin

// Motor 2 pin usage
#define PWMpinMotorRight 16     // PWM pin
#define DIRpinMotorRight 4      // DIR pin
#define encoderD3MotorRight 13  // Encoder D3 pin
#define encoderD2MotorRight 14  // Encoder D2 pin

// ---------- Declare class for encoder ----------

// Measure motor left
ESP32Encoder encoderLeft;
// Measure motor right
ESP32Encoder encoderRight;

// ---------- Declare usage variable ----------

// Create setup motor driver board
// !! Warning !! If higher pwm_resolution -> lower pwm_frequency. If lower pwm_resolution -> higher pwm_frequency. ( Calculate on this equation f_pwm = f_clock/2^^resolution )
// Max choice => ( PWM_FREQUENCY = 312kHz, PWM_RESOLUTION = 8bit )( PWM_FREQUENCY = 78kHz, PWM_RESOLUTION = 10bit )( PWM_FREQUENCY = 19kHz, PWM_RESOLUTION = 12bit )( PWM_FREQUENCY = 1.2kHz, PWM_RESOLUTION = 16bit )( PWM_FREQUENCY = 76Hz, PWM_RESOLUTION = 20bit )
const int PWM_FREQUENCY = 10000;  // Quite motor with more frequency in one duty cycle.
const int PWM_RESOLUTION = 10;   // This pwm pinout limit with 20 bit

// Create global variable storage of robot mode in type of int
int MODE_ROBOT = 0;

// Define the PID variable for tuning motor
double SETPOINT_LEFT = 0.0, INPUT_LEFT = 0.0, OUTPUT_LEFT = 0.0, SETPOINT_RIGHT = 0.0, INPUT_RIGHT = 0.0, OUTPUT_RIGHT = 0.0;
// Define the aggressive and conservative Tuning parameter
double consKp = 3.5, consKi = 2.0, consKd = 0.001;          // Inside the steady state
double aggKp = 6.0, aggKi = 5.0, aggKd = 0.0;               // Before the steady state
// Declare the ticking count encoder ( avoid tick with fast hz )
unsigned long currtick = 0;

// Create the subscription value speed motor from /cmd_vel
int16_t subscription_speed_left = 0;
int16_t subscription_speed_right = 0;

// Declare the robot width(meter) from lwheel_joint to rwheel_joint in xacro 
float WIDTH_ROBOT = 0.398;
float WHEEL_DIAMETER = 0.123;



// !!!!!!!!!! Dummy /cmd_vel including ( linear velocity and angular velocity ) !!!!!!!!!!
float LINEAR_VELOCITY = 0.0;
float ANGULAR_VELOCITY = 0.0;

// Declare the variable for PRM and velocity
float valuePulsePerSec_left = 0.0;
float valueRPM_left = 0.0;
float valueVelocity_left = 0.0;

// Declare the pulse per revolute of full speed
const int PPR = 480; 

// Declare the timer value
unsigned long lastTime, previousTime, timeSth;

// Define the pin to toggle similar as the switch
#define toggleSimilarSwitch 21   // GPIO Input

// ---------- PID initial tuning parameters ----------

// Specify the links and initial tuning parameters
PID_v2 PID_Left_Motor(consKp, consKi, consKd, PID::Direct);
PID_v2 PID_Right_Motor(consKp, consKi, consKd, PID::Direct);

// ---------- Restart robot ----------
// When the time was reach 1 hours 30 min cut down all action.

// ---------- Additional function ----------

void createLinearAndAngularVelocityWithDummy( float dummy_linear_velocity, float dummy_angular_velocity ){
  // Create variable for caluculate diff drive
  float convert_cmd_vel_left, convert_cmd_vel_right; 

  // Convert the Twist velocity m/s into speed of each motor velocity by using diff drive equation
  // diff drive equation -> Vr = v + (L/2)w, Vl = v - (L/2)w
  convert_cmd_vel_left = ( dummy_linear_velocity ) - ( ( WIDTH_ROBOT / 2 )* dummy_angular_velocity );
  convert_cmd_vel_right = ( dummy_linear_velocity ) + ( ( WIDTH_ROBOT / 2 )* dummy_angular_velocity );

  // Limit the velocity of pwm_resolution 15 bit motor ( linear limit = 0.75 m/s, angular limit = xxx rad/s )( single channel 11904 pulse = 1.59719 m/s, full quadrature 47616 pulse = 1.59719 m/s )
  // subscription_speed_left = constrain( convert_cmd_vel_left * SCALE_ACCELERATE_VELOCITY, -500 , 500 );
  // subscription_speed_right = constrain( convert_cmd_vel_right * SCALE_ACCELERATE_VELOCITY, -500 , 500 );

  // Convert the speed m/s in each motor into pulse/sec( but this code calculate in term of 100ms, so divine by 10 )
  subscription_speed_left = ( ( convert_cmd_vel_left * PPR ) / ( 2 * PI * ( WHEEL_DIAMETER / 2 ) ) ) / 10;
  subscription_speed_right = ( ( convert_cmd_vel_right * PPR ) / ( 2 * PI * ( WHEEL_DIAMETER / 2 ) ) ) / 10;


  // Disable motor right
  subscription_speed_right = 0;




  // Serial.print( "subscription_speed_left : " );
  // Serial.println( subscription_speed_left );
  // Serial.print( "subscription_speed_right : " );
  // Serial.println( subscription_speed_right );

  // Set the end plotter
  // if( millis() - previousTime > 100 ){
  //   previousTime = millis();
  //   float endPlotter_velocity = subscription_speed_left;
  //   Serial.print( "Variable_4:" );
  //   Serial.println( endPlotter_velocity );
  // }
  float endPlotter_velocity = subscription_speed_left;
  Serial.print( "Variable_4:" );
  Serial.println( endPlotter_velocity );

}

// Create function for control each motor.
void setMotorSpeed(int DIRPinInput, int PWMPinInput, int speedMotor) {
  // Create limit speed of Motor, the maximum was base on PWM_RESOLUTION.( Generate PWM )
  // The max speed was 1.59719 m/s( 100% duty cycle ). But in this case, We use speed limit 0.75 m/s( 48% duty cycle )
  int limitSpeed = constrain(speedMotor, -500, 500);

  // Check it was positive num => Motor will move forward.
  if (limitSpeed > 0) {
    // Set direction to move forward.
    digitalWrite(DIRPinInput, 1);
    // Generate Pulse Width Modulation to motor.
    ledcWrite(PWMPinInput, limitSpeed);
  }
  // Check it was negative num => Motor will move backward.
  else if (limitSpeed < 0) {
    // Set direction to move backward.
    digitalWrite(DIRPinInput, 0);
    // Generate Pulse Width Modulation to motor.
    ledcWrite(PWMPinInput, -limitSpeed);
  }
  // Check it was zero => Motor will stop.
  else if (limitSpeed == 0) {
    // Set direction to move backward.
    digitalWrite(DIRPinInput, 1);
    // Generate Pulse Width Modulation to motor.
    ledcWrite(PWMPinInput, 0);
  }
}

// Create function for adjust the aggressive PID value and constance PID value 
void PIDEncoderAdaptiveGap(){
  // Sample every 100ms ( This function for convert into sample pulse without constance velocity )
  // ( to avoid decimal calculate in pulse but need to relate with pulse per revolute of motor )
  // ( 100 mean use only 100/max to define the else of the pulse sample )
  if( millis() - currtick > 100 ){
    currtick = millis();

    // Assign value from encoder to input PID
    INPUT_LEFT = abs( encoderLeft.getCount() );
    INPUT_RIGHT = abs( encoderRight.getCount() );
    // Clear the first to start with 0 value
    encoderRight.clearCount();
    encoderLeft.clearCount();
  }

  // Set the new setpoint in PID ( using abs because it was calculate to scalar value only don't need to define the direction in PID )
  PID_Left_Motor.Setpoint( abs( subscription_speed_left ) );
  PID_Right_Motor.Setpoint( abs( subscription_speed_right ) );

  // // Create measure distance away from setpoint left motor
  double gapLeftMotor = abs( PID_Left_Motor.GetSetpoint() - INPUT_LEFT );

  // Check if the gap of the setpoint on left motor was less than requirement
  if ( gapLeftMotor < 5 ){
    // we're close to setpoint, use conservative tuning parameters
    PID_Left_Motor.SetTunings( consKp, consKi, consKd );
  } else {
    // we're far from setpoint, use aggressive tuning parameters
    PID_Left_Motor.SetTunings( aggKp, aggKi, aggKd );
  }

  // Create measure distance away from setpoint right motor
  double gapRightMotor = abs( PID_Right_Motor.GetSetpoint() - INPUT_RIGHT );

  // Check if the gap of the setpoint on left motor was less than requirement
  if ( gapLeftMotor < 5 ){
    // we're close to setpoint, use conservative tuning parameters
    PID_Right_Motor.SetTunings( consKp, consKi, consKd );
  } else {
    // we're far from setpoint, use aggressive tuning parameters
    PID_Right_Motor.SetTunings( aggKp, aggKi, aggKd );
  }



  // !!!!! Set only aggessive or conservative parameters !!!!!
  // PID_Left_Motor.SetTunings( consKp, consKi, consKd );
  // PID_Right_Motor.SetTunings( consKp, consKi, consKd );


  // Run the pid function 
  OUTPUT_LEFT = PID_Left_Motor.Run( INPUT_LEFT );
  OUTPUT_RIGHT = PID_Right_Motor.Run( INPUT_RIGHT );

  // if( millis() - timeSth > 100 ){
  //   timeSth = millis();
  //   Serial.print( "OUTPUT_LEFT : " );
  //   Serial.println( OUTPUT_LEFT );
  //   Serial.print( "OUTPUT_RIGHT : " );
  //   Serial.println( OUTPUT_RIGHT );
  // }
  Serial.print( "OUTPUT_LEFT : " );
  Serial.println( OUTPUT_LEFT );
  Serial.print( "OUTPUT_RIGHT : " );
  Serial.println( OUTPUT_RIGHT );

}

// Create function for control motor with the navigation method
void navigationMotorControl(){

  // Setting the init velocity and angular velocity
  // setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 0);
  // setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, 0);

  // Call the function PID adaptive gap
  PIDEncoderAdaptiveGap();

  // Serial.print( "OUTPUT_LEFT : " );
  // Serial.println( OUTPUT_LEFT );
  // Serial.print( "OUTPUT_RIGHT : " );
  // Serial.println( OUTPUT_RIGHT );


  // Check the direction of motor left and control it 
  if( subscription_speed_left > 0 ){
    // Call the function setMotorSpeed using value OUTPUT_LEFT
    setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, OUTPUT_LEFT );
  } else if ( subscription_speed_left < 0 ){
    // Call the function setMotorSpeed using value OUTPUT_LEFT
    setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, -1*OUTPUT_LEFT );
  } else {
    // Call the function setMotorSpeed using value OUTPUT_LEFT
    setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 0 );
  }

  // Check the direction of motor right and control it 
  if( subscription_speed_right > 0 ){
    // Call the function setMotorSpeed using value OUTPUT_LEFT
    setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, OUTPUT_RIGHT );
  } else if ( subscription_speed_left < 0 ){
    // Call the function setMotorSpeed using value OUTPUT_LEFT
    setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, -1*OUTPUT_RIGHT );
  } else {
    // Call the function setMotorSpeed using value OUTPUT_LEFT
    setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, 0 );
  }
}

// Create output function for visualize the encoder position
void displayValue(){
	// Print serial count (If you need to use Serial Plotter need to use Serial.print( "Variable_xx" ) first to define value in plotter)
	Serial.print( "Encoder motor left ( pulse per sec ): " );
  Serial.print( "Variable_1:" );
  valuePulsePerSec_left = ( INPUT_LEFT );         // ( *10 come from convert milisecond -> second, *4 come from convert single channel -> full quadrature  )
  Serial.println( valuePulsePerSec_left );
  // Serial.print( "Encoder motor right (unit pulse per sec): " );
  // Serial.print( "Variable_2:" );
  // Serial.println( INPUT_RIGHT * 10 );

  // Serial.print( "Encoder motor left ( RPM ): " );
  valueRPM_left = ( valuePulsePerSec_left / PPR ) * 60;
  // Serial.println( valueRPM_left );

  // Serial.print( "Encoder motor left ( velocity ): " );
  valueVelocity_left = ( valueRPM_left / 60 ) * ( 2 * PI * 0.123 );
  // Serial.print( "Variable_1:" );
  // Serial.println( valueVelocity_left );

  // Set the start plotter
  float startPlotter_velocity = 0;
  Serial.print( "Variable_3:" );
  Serial.println( startPlotter_velocity );
  // Set the end plotter
  // float endPlotter_velocity = 1200;
  // Serial.print( "Variable_4:" );
  // Serial.println( endPlotter_velocity );
}


// ---------- Setup robot ----------

// Function for setup encoder of motor.
void encoderSetup() {
  // Setup motor encoder with pullup
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // Setup motor left
  encoderLeft.attachFullQuad(encoderD3MotorLeft, encoderD2MotorLeft);
  encoderLeft.clearCount();

  // Setup motor right
  encoderRight.attachFullQuad(encoderD3MotorRight, encoderD2MotorRight);
  encoderRight.clearCount();

  // Set count value to 0
  encoderLeft.setCount(0);
  encoderRight.setCount(0);
}

// Function for setup motor and generate PWM.
void motorAndLedcSetup() {
  // Setup motor( ledcLibrary )
  pinMode(DIRpinMotorLeft, OUTPUT);
  pinMode(DIRpinMotorRight, OUTPUT);

  // Setup init motor
  ledcAttach(PWMpinMotorLeft, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWMpinMotorRight, PWM_FREQUENCY, PWM_RESOLUTION);
}

// Function for setup board NodeMCU-32S.
void basicSetup() {
  // Setting baud rate to 115200
  Serial.begin(115200);

  // Setup pin input
  pinMode( toggleSimilarSwitch, INPUT );
}

void PIDSetup(){
  // Setup the PID including( input, current output, and setpoint )
  PID_Left_Motor.Start( 0, 0, 0 );
  PID_Right_Motor.Start( 0, 0, 0 );

  // Setup the output limit from PID ( Default = [ 0, 255 ] )
  PID_Left_Motor.SetOutputLimits( 0, 1023 );
  PID_Right_Motor.SetOutputLimits( 0, 1023 );
}

// ---------- Main ----------

void setup() {
  // put your setup code here, to run once:

  // Call the function to all setup 
  encoderSetup();
  motorAndLedcSetup();
  basicSetup();
  PIDSetup();
}

long lastCount = 0;
void loop() {
  // put your main code here, to run repeatedly:

  // -------------------- Test code 1 --------------------
  // Create the repeat code to test read the encoder with the limit value
  // setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 500);
  // if( millis() - lastTime > 1000 ){
  //   lastTime = millis();

  //   // Assign value from encoder to input PID
  //   INPUT_RIGHT = abs( encoderRight.getCount() );
  //   INPUT_LEFT = abs( encoderLeft.getCount() );
  //   // Clear the first to start with 0 value
  //   encoderRight.clearCount();
  //   encoderLeft.clearCount();

  //   // Display the velocity of left motor.
  //   displayValue();
  // }
  // -------------------- Test code 1 --------------------


  // -------------------- Test code 2 --------------------
  // // Create the repeat code to test read the encoder with the limit value
  // setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 0);
  // unsigned long now = millis();
  // if( now - lastTime > 1000 ){
  //   INPUT_LEFT = encoderLeft.getCount();
  //   long deltaCount = INPUT_LEFT - lastCount;

  //   float deltaTime = ( now -lastTime ) / 1000.0;

  //   float rpm = ( deltaCount / (float)PPR ) * ( 60.0 / deltaTime );

  //   Serial.print( "RPM : " );
  //   Serial.println( rpm );

  //   // Display the velocity of left motor.
  //   displayValue();
  // }
  // -------------------- Test code 2 --------------------

  // -------------------- Test code 3 --------------------
  // if( millis() - lastTime > 50000 ){
  //   lastTime = millis();

  //   // Assign value from encoder to input PID
  //   INPUT_RIGHT = abs( encoderRight.getCount() );
  //   INPUT_LEFT = abs( encoderLeft.getCount() );


  //   // Display the velocity of left motor.
  //   displayValue();

  //   // Clear the first to start with 0 value
  //   encoderRight.clearCount();
  //   encoderLeft.clearCount();

  // }
  // -------------------- Test code 3 --------------------
  // Create switch velocity
  if ( digitalRead( toggleSimilarSwitch ) == 1 ){
    LINEAR_VELOCITY = 0.75;
    ANGULAR_VELOCITY = 0.0;
  } else {
    LINEAR_VELOCITY = 0.0;
    ANGULAR_VELOCITY = 0.0;
  }
  // Convert input linear and angular velocity into speed motor left and right.
  createLinearAndAngularVelocityWithDummy( LINEAR_VELOCITY, ANGULAR_VELOCITY );

  // Call the function, Control the velocity using PID in left_motor and right_motor.
  navigationMotorControl();

  // Display the pulse / sec ( in this term we have use 100ms )
  displayValue();
  
}
