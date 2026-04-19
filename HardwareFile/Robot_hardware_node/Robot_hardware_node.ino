// ---------- Import library ----------
// Import library for communicate micro-ROS and ESP32.
#include <micro_ros_arduino.h>

// Import library standard C input/output library.
#include <stdio.h>

// Core ROS2 C client library ( low-level ROS2 function )
#include <rcl/rcl.h>

// ROS2 error handling utilities.
#include <rcl/error_handling.h>

// rclc = simplified ROS2 API designed for microcontrollers.
#include <rclc/rclc.h>

// Executor handles callback( timers, subscription, etc. )
#include <rclc/executor.h>

// Import ROS2 message type.
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int16.h>

// Import the library for reading the encoder position
#include <ESP32Encoder.h>

// Import the core API function( pinMode(), digitalWrite(), etc ), constants and macros( HIGH, LOW, min(), max() )
#include <Arduino.h>

// Import the library for using PID control
#include <PID_v2.h>

// Import the library for using mathematic
#include <math.h>

// ---------- ROS object declarations ----------

// Create subscriber object and message type.( subscribe => cmd_vel )
rcl_subscription_t subscriberTwist;
geometry_msgs__msg__Twist msg_twist_cmd_vel;

// Create subscriber object and message type.( subscribe => mode_robot )
rcl_subscription_t subscriberMode;
std_msgs__msg__Int8 msg_int8_mode;

// Create publisher object and message type.( publish => error meassge )
rcl_publisher_t publisherError;
std_msgs__msg__String msg_string_error;

// Create publisher object and message type.( publish => status meassge )
rcl_publisher_t publisherStatus;
std_msgs__msg__String msg_string_status;



// TUTA DEBUGGGGGGG
rcl_publisher_t publisherDebug;
std_msgs__msg__Int8 msg__debug;



// Create publisher object and message type.( publish => left wheel encoder tick data )
rcl_publisher_t publisherTickLeftEncoder;
std_msgs__msg__Int16 msg_int16_left_tick_encoder;

// Create publisher object and message type.( publish => right wheel encoder tick data )
rcl_publisher_t publisherTickRightEncoder;
std_msgs__msg__Int16 msg_int16_right_tick_encoder;

// Create executor which will run callbacks( timers, subscriptions ) when new data arrives.
rclc_executor_t executor_subscriber_Twist;
rclc_executor_t executor_subscriber_Int8;
rclc_executor_t executor_publisher;

// Structure containing ROS2 initialization and context information.
rclc_support_t support;

// ROS2 memory allocator.
rcl_allocator_t allocator;

// Create ROS2 node object.
rcl_node_t node;

// Create ROS2 timer object. ( used to trigger periodic callbacks )
rcl_timer_t timer;

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

// Measure motor left( in term of pulse/sec )
ESP32Encoder encoderLeft;
// Measure motor right( in term of pulse/sec )
ESP32Encoder encoderRight;

// Measure motor left( in term of accumulate value )
ESP32Encoder encoderTickLeft;
// Measure motor right( in term of accumulate value )
ESP32Encoder encoderTickRight;

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

// Declare the robot specification of the robot.
float WIDTH_ROBOT = 0.398;
float WHEEL_DIAMETER = 0.123;

// Declare the pulse per revolute of full speed
const int PPR = 480; 

// Create variable for caluculate diff drive equation
float convert_cmd_vel_left = 0.0; 
float convert_cmd_vel_right = 0.0;
// Create the subscription value speed motor from /cmd_vel
int16_t SUBSCRIPTION_SPEED_LEFT = 0;
int16_t SUBSCRIPTION_SPEED_RIGHT = 0;

// ---------- PID initial tuning parameters ----------

// Specify the links and initial tuning parameters
PID_v2 PID_Left_Motor(consKp, consKi, consKd, PID::Direct);
PID_v2 PID_Right_Motor(consKp, consKi, consKd, PID::Direct);

// ---------- Error checking function ----------

// RCCHECK runs a ROS function and enters error_loop if the function fails. ( rcl_ret_t => return complete, failed, timeout etc. Simplified to excepation in python )
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_notice(); } \
  }

// RCSOFTCHECK runs a ROS function and also goes to error_loop if it fails.
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_notice(); } \
  }

// Create function for notice the error appear.
void error_notice() {
  // If the code was error assign to global variable.
  msg_string_error.data.data = "There was some error, So It was RCCHECK and RCSOFTCHECK error.";
}

// ---------- Callback function ----------

// Create function for subscribe the int8 message.( Mode of robot )
void subscriberInt8_callback(const void *msgin) {
  // Declare the valiable to storage the mode of robot from subscriber. ( Convert generic pointer ( msgin ) into Int8 message pointer )
  const std_msgs__msg__Int8 *msg_int8_mode = (const std_msgs__msg__Int8 *)msgin;

  // Assign the msgin into global variable robot mode
  MODE_ROBOT = ( msg_int8_mode->data );

}

// Create function for subscribe the cmd_vel message.
void subscriberTwist_callback(const void *msgin) {
  // Declare the valiable to storage the cmd_vel from subscriber. ( Convert generic pointer ( msgin ) into a Twist message pointer )
  const geometry_msgs__msg__Twist *msg_twist_cmd_vel = (const geometry_msgs__msg__Twist *)msgin;

  // Convert the Twist velocity m/s into speed of each motor velocity by using diff drive equation
  // diff drive equation -> Vr = v + (L/2)w, Vl = v - (L/2)w
  convert_cmd_vel_left = ( msg_twist_cmd_vel->linear.x ) - ( ( WIDTH_ROBOT / 2 ) * msg_twist_cmd_vel->angular.z );
  convert_cmd_vel_right = ( msg_twist_cmd_vel->linear.x ) + ( ( WIDTH_ROBOT / 2 ) * msg_twist_cmd_vel->angular.z );
  
  // Convert the speed m/s in each motor into pulse/sec( but this code calculate in term of 100ms, so divine by 10 )
  SUBSCRIPTION_SPEED_LEFT = ( ( convert_cmd_vel_left * PPR ) / ( 2 * PI * ( WHEEL_DIAMETER / 2 ) ) ) / 10;
  SUBSCRIPTION_SPEED_RIGHT = ( ( convert_cmd_vel_right * PPR ) / ( 2 * PI * ( WHEEL_DIAMETER / 2 ) ) ) / 10;


  msg__debug.data = SUBSCRIPTION_SPEED_LEFT;
  
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  // Prevent complier warning. ( variable not used )
  RCLC_UNUSED(last_call_time);

  // Make sure timer exists.
  if (timer != NULL) {

    // Call function to recieve the tick encoder value 
    getTickEncoderValue();

    // Publish the error message on ROS2 topic.
    RCSOFTCHECK(rcl_publish(&publisherError, &msg_string_error, NULL));
    // Publish the status message on ROS2 topic.
    RCSOFTCHECK(rcl_publish(&publisherStatus, &msg_string_status, NULL));
    // Publish the left tick encoder message on ROS2 topic.
    RCSOFTCHECK(rcl_publish(&publisherTickLeftEncoder, &msg_int16_left_tick_encoder, NULL));  
    // Publish the right tick encoder message on ROS2 topic.
    RCSOFTCHECK(rcl_publish(&publisherTickRightEncoder, &msg_int16_right_tick_encoder, NULL));  


    // TUTA DEBUGGGGGGGG
    RCSOFTCHECK(rcl_publish(&publisherDebug, &msg__debug, NULL));
    
  }
}

// ---------- Restart robot ----------
// When the time was reach 1 hours 30 min cut down all action.

// ---------- Additional function ----------

// Create function for control each motor.
void setMotorSpeed(int DIRPinInput, int PWMPinInput, int speedMotor) {
  // Create limit speed of Motor, the maximum was base on PWM_RESOLUTION.( Generate PWM )
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
    INPUT_RIGHT = abs( encoderRight.getCount() );
    INPUT_LEFT = abs( encoderLeft.getCount() );
    // Clear the first to start with 0 value
    encoderRight.clearCount();
    encoderLeft.clearCount();
  }

  // Set the new setpoint in PID ( using abs because it was calculate to scalar value only don't need to define the direction in PID )
  PID_Left_Motor.Setpoint( abs( SUBSCRIPTION_SPEED_LEFT ) );
  PID_Right_Motor.Setpoint( abs( SUBSCRIPTION_SPEED_RIGHT ) );

  // Create measure distance away from setpoint left motor
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

  // Run the pid function 
  OUTPUT_LEFT = PID_Left_Motor.Run( INPUT_LEFT );
  OUTPUT_RIGHT = PID_Right_Motor.Run( INPUT_RIGHT );
}

// Create function for control motor with the navigation method
void navigationMotorControl(){
  // Send out status control the robot.
  msg_string_status.data.data = "Start control robot using navigation.";

  // Call the function PID adaptive gap
  PIDEncoderAdaptiveGap();

  // Check the direction of motor left and control it 
  if( SUBSCRIPTION_SPEED_LEFT > 0 ){
    // Call the function setMotorSpeed using value OUTPUT_LEFT
    setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, OUTPUT_LEFT );
  } else if ( SUBSCRIPTION_SPEED_LEFT < 0 ){
    // Call the function setMotorSpeed using value OUTPUT_LEFT
    setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, -1*OUTPUT_LEFT );
  } else {
    // Call the function setMotorSpeed using value OUTPUT_LEFT
    setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 0 );
  }

  // Check the direction of motor right and control it 
  if( SUBSCRIPTION_SPEED_RIGHT > 0 ){
    // Call the function setMotorSpeed using value OUTPUT_LEFT
    setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, OUTPUT_RIGHT );
  } else if ( SUBSCRIPTION_SPEED_LEFT < 0 ){
    // Call the function setMotorSpeed using value OUTPUT_LEFT
    setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, -1*OUTPUT_RIGHT );
  } else {
    // Call the function setMotorSpeed using value OUTPUT_LEFT
    setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, 0 );
  }
}

// Create function for recieve cmd_vel to control robot
void controlRobotTeleop() {
  // Send out status control the robot.
  msg_string_status.data.data = "Start control robot using teleop.";

  // Check, If value of linear X was positive. ( Robot move forward )
  if ( ( SUBSCRIPTION_SPEED_LEFT > 0) && ( SUBSCRIPTION_SPEED_RIGHT > 0) ) {
    setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 500);
    setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, 500);
  }
  // Check, If value of linear X was negative. ( Robot move backward )
  else if ( ( SUBSCRIPTION_SPEED_LEFT < 0 ) && ( SUBSCRIPTION_SPEED_RIGHT < 0 ) ) {
    setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, -500);
    setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, -500);
  }
  // Check, If value of linear X was negative. ( Robot turn right )
  else if ( ( SUBSCRIPTION_SPEED_LEFT > 0 ) && ( SUBSCRIPTION_SPEED_RIGHT < 0 ) ) {
    setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, -500);
    setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, 500);
  }
  // Check, If value of linear X was negative. ( Robot move left )
  else if ( ( SUBSCRIPTION_SPEED_LEFT < 0 ) && ( SUBSCRIPTION_SPEED_RIGHT > 0 ) ) {
    setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 500);
    setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, -500);
  }
  // If there was't any match condition one of above.
  else {
    setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 0);
    setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, 0);
  }
}

// Create function for recieve the tick accumulate value 
void getTickEncoderValue(){
  // Count the tick encoder
  msg_int16_left_tick_encoder.data = encoderTickLeft.getCount();
  msg_int16_right_tick_encoder.data = encoderTickRight.getCount();
}

// ---------- Setup robot ----------

// Function for setup micro-ROS.
void microROSSetup() {
  // Initialize micro-ROS communication transport. ( Serial, WiFi, or UDP depending on configuration )
  set_microros_transports();

  // Assign the default ROS2 memory allocator.
  allocator = rcl_get_default_allocator();

  // Initialize ROS2 support structure.
  // Initializes ROS2 system and context ( create this function before creating nodes, publishers, subscribers, etc. )( function prototype => rcl_ret_t rclc_support_init(rclc_support_t * support, int argc, const char * const * argv, rcl_allocator_t * allocator ); )
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create ROS2 node.
  RCCHECK(rclc_node_init_default(
    &node,                   // Node object
    "micro_ROS_ESP32_node",  // Node name
    "",                      // Namespace
    &support));              // Support structure

  // Create ROS2 subscriber. ( subscribe mode message )
  RCCHECK(rclc_subscription_init_default(
    &subscriberMode,                                   // Subscriber int8 object
    &node,                                             // Node which owns the subscriber
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),  // Message type
    "micro_ros_mode_robot"));                          // Topic name

  // Create ROS2 subscriber. ( subscribe cmd_vel message )
  RCCHECK(rclc_subscription_init_default(
    &subscriberTwist,                                        // Subscriber twist object
    &node,                                                   // Node that owns the subscriber
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),  // Message type
    "micro_ros_cmd_vel"));                                   // Topic name

  // Create ROS2 publisher.( publish error message )
  RCCHECK(rclc_publisher_init_default(
    &publisherError,                                     // Publisher error object
    &node,                                               // Node that owns the publisher
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),  // Message type
    "micro_ros_error_message"));                         // Topic name

  // Create ROS2 publisher.( publish status message )
  RCCHECK(rclc_publisher_init_default(
    &publisherStatus,                                    // Publisher error object
    &node,                                               // Node that owns the publisher
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),  // Message type
    "micro_ros_status_message"));                        // Topic name

  // Create ROS2 publisher.( publish left wheel encoder tick data message )
  RCCHECK(rclc_publisher_init_default(
    &publisherTickLeftEncoder,                           // Publisher error object
    &node,                                               // Node that owns the publisher
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),   // Message type
    "micro_ros_left_enc_tick_data"));                    // Topic name
  
  // Create ROS2 publisher.( publish right wheel encoder tick data message )
  RCCHECK(rclc_publisher_init_default(
    &publisherTickRightEncoder,                           // Publisher error object
    &node,                                               // Node that owns the publisher
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),   // Message type
    "micro_ros_right_enc_tick_data"));                    // Topic name

  // TUTA DEBUGGGGGGGGGGGGGGGGg
  RCCHECK(rclc_publisher_init_default(
    &publisherDebug,                                   // Publisher error object
    &node,                                             // Node that owns the publisher
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),  // Message type
    "DEBUG_topic"));                                   // Topic name

  // Create timer callback.
  // Create publish rate of message ( 1000ms = 1hz, 500ms = 2hz, 100ms = 10hz, 50ms = 20hz, 10ms = 100hz )
  const unsigned int timer_timeout = 20;  // timer period in milliseconds (1000 ms = 1 second = 1hz)
  RCCHECK(rclc_timer_init_default(
    &timer,                       // Timer object
    &support,                     // Support object
    RCL_MS_TO_NS(timer_timeout),  // Convert milliseconds to nanoseconds to assign timeout
    timer_callback));             // Call function when timer triggers

  // Executor create and assign the them.( This function will call when the new data arrives )
  // Create executor for subscriber.
  RCCHECK(rclc_executor_init(
    &executor_subscriber_Twist,  // Executor subscriber object
    &support.context,      // ROS2 context
    1,                     // number of handles ( one subscription )
    &allocator));          // memory allocator
  RCCHECK(rclc_executor_init(
    &executor_subscriber_Int8,  // Executor subscriber object
    &support.context,      // ROS2 context
    1,                     // number of handles ( one subscription )
    &allocator));          // memory allocator
  // Add subscription to executor. So, This function will call when the new data arrives.
  RCCHECK(rclc_executor_add_subscription(
    &executor_subscriber_Twist,      // Executor subscriber object
    &subscriberMode,           // Subscriber int8 object
    &msg_int8_mode,            // Message storage variable
    &subscriberInt8_callback,  // Callback triggered when new message arrives
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor_subscriber_Int8,       // Executor subscriber object
    &subscriberTwist,           // Subscriber twist object
    &msg_twist_cmd_vel,         // Message storage variable
    &subscriberTwist_callback,  // Subscriber callback
    ON_NEW_DATA));              // Callback triggered when new message arrives

  // Create executor for publisher
  RCCHECK(rclc_executor_init(
    &executor_publisher,  // Executor publisher object
    &support.context,     // ROS context
    1,                    // Number of handles( only one timer here )
    &allocator));         // Memory allocator
  // Add timer to executor so it can excute timer_callback
  RCCHECK(rclc_executor_add_timer(&executor_publisher, &timer));
}

// Function for setup encoder of motor.
void encoderSetup() {
  // Setup motor encoder with pullup
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // Setup motor left
  encoderLeft.attachFullQuad(encoderD3MotorLeft, encoderD2MotorLeft);
  encoderLeft.clearCount();
  encoderTickLeft.attachFullQuad(encoderD3MotorLeft, encoderD2MotorLeft);
  encoderTickLeft.clearCount();

  // Setup motor right
  encoderRight.attachFullQuad(encoderD3MotorRight, encoderD2MotorRight);
  encoderRight.clearCount();
  encoderTickRight.attachFullQuad(encoderD3MotorRight, encoderD2MotorRight);
  encoderTickRight.clearCount();

  // Set count value to 0
  encoderLeft.setCount(0);
  encoderRight.setCount(0);
  encoderTickLeft.setCount(0);
  encoderTickRight.setCount(0);
}

// Function for setup motor and generate PWM.
void motorAndLedcSetup() {
  // Setup motor( ledcLibrary )
  pinMode(DIRpinMotorLeft, OUTPUT);
  pinMode(DIRpinMotorRight, OUTPUT);

  // Setup init motor
  ledcAttach(PWMpinMotorLeft, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWMpinMotorRight, PWM_FREQUENCY, PWM_RESOLUTION);

  // Setting the init velocity and angular velocity
  setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 0);
  setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, 0);
}

// Function for setup board NodeMCU-32S.
void basicSetup() {
  // Setting baud rate to 115200
  Serial.begin(115200);
}

void PIDSetup(){
  // Setup the output limit from PID refer limit from pwm_resolution ( Default = [ 0, 255 ] )
  PID_Left_Motor.SetOutputLimits( 0, 1023 );
  PID_Right_Motor.SetOutputLimits( 0, 1023 );

  // Setup the PID including( input, current output, and setpoint )
  PID_Left_Motor.Start( 0, 0, 0 );
  PID_Right_Motor.Start( 0, 0, 0 );
}

// ---------- Main ----------

void setup() {
  // put your setup code here, to run once:

  // Call the function to setup micro-ROS
  microROSSetup();
  // Call the function to setup encoder
  encoderSetup();
  // Call the function to setup motor and pwm pin
  motorAndLedcSetup();
  // Call the function to setup basic arduino ide
  basicSetup();
  // Call the function to setup PID
  PIDSetup();
}

void loop() {
  // put your main code here, to run repeatedly:

  // Spin the node to run subscriber Twist repeatedly
  // executor frequency >= 3-5x timer frequency ( Think of it like this: Timer = when data should be sent (50 Hz), Executor = how often you check for work )( If executor is too slow: Messages get delayed, Timing becomes unstable )
  RCCHECK(rclc_executor_spin_some(
    &executor_subscriber_Twist,  // Executor subscriber object
    RCL_MS_TO_NS(1)));   // Allow excutor to run 10 hz

  // Spin the node to run subscriber Int8 repeatedly
  // executor frequency >= 3-5x timer frequency ( Think of it like this: Timer = when data should be sent (50 Hz), Executor = how often you check for work )( If executor is too slow: Messages get delayed, Timing becomes unstable )
  RCCHECK(rclc_executor_spin_some(
    &executor_subscriber_Int8,  // Executor subscriber object
    RCL_MS_TO_NS(1)));   // Allow excutor to run 10 hz

  // Spin the node to run publisher repeatedly
  // executor frequency >= 3-5x timer frequency ( Think of it like this: Timer = when data should be sent (50 Hz), Executor = how often you check for work )( If executor is too slow: Messages get delayed, Timing becomes unstable )
  RCCHECK(rclc_executor_spin_some(
    &executor_publisher,  // Executro publisher object
    RCL_MS_TO_NS(1)));  // Allow excutor to run 10 hz

  // Check the mode robot to select the control mode
  if( MODE_ROBOT == 1 ){
    // Get in the function of control robot by automation.
    navigationMotorControl();

  } else if( MODE_ROBOT == 2 ){
    // Get in the function of control robot by teleop.
    controlRobotTeleop();

  } else{
    // Debug if user didn't define the MODE_ROBOT
    msg_string_status.data.data = "There wasn't match MODE_ROBOT or MODE_ROBOT was empty.";
  }
}
