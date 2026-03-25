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

// Import the library for reading the encoder position
#include <ESP32Encoder.h>

// Import the core API function( pinMode(), digitalWrite(), etc ), constants and macros( HIGH, LOW, min(), max() )
#include <Arduino.h>

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

// Measure motor left
ESP32Encoder encoderLeft;
// Measure motor right
ESP32Encoder encoderRight;

// ---------- Declare usage variable ----------

// Create setup motor driver board
const int PWM_FREQUENCY = 20000;  // Quite motor with more frequency in one duty cycle.
const int PWM_RESOLUTION = 8;     // The maximum resolution to input motor speed. ( max 12 bit ) -> 0[min speed adjust]-255[max speed adjust] = 0[min speed adjust]-4095[max speed adjust]

// Create global variable storage of robot mode in type of int
int MODE_ROBOT = 0;


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


  // Get in the function of control robot, If it was contorl robot mode.
  controlRobotTeleop( msg_twist_cmd_vel->linear.x, msg_twist_cmd_vel->angular.z, MODE_ROBOT );
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  // Prevent complier warning. ( variable not used )
  RCLC_UNUSED(last_call_time);

  // Make sure timer exists.
  if (timer != NULL) {

    // Publish the error message on ROS2 topic.
    RCSOFTCHECK(rcl_publish(&publisherError, &msg_string_error, NULL));
    // Publish the status message on ROS2 topic.
    RCSOFTCHECK(rcl_publish(&publisherStatus, &msg_string_status, NULL));


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
  int limitSpeed = constrain(speedMotor, -255, 255);

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

// Create function for recieve cmd_vel to control robot
void controlRobotTeleop( int cmdVelLinearX, int cmdVelAngularZ, int modeOfRobot ) {
  // Send out status control the robot.
  msg_string_status.data.data = "Start control robot using teleop.";

  msg__debug.data = modeOfRobot;

  // Setting the init velocity and angular velocity
  setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 0);
  setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, 0);

  // Check if it was in control robot mode
  if ( modeOfRobot == 2 ) {
    
    // Check, If value of linear X was positive. ( Robot move forward )
    if ((cmdVelLinearX > 0) && (cmdVelAngularZ == 0)) {
      setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 100);
      setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, 100);
    }
    // Check, If value of linear X was negative. ( Robot move backward )
    else if ((cmdVelLinearX < 0) && (cmdVelAngularZ == 0)) {
      setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, -100);
      setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, -100);
    }
    // Check, If value of linear X was negative. ( Robot turn left )
    else if ((cmdVelLinearX == 0) && (cmdVelAngularZ < 0)) {
      setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, -100);
      setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, 100);
    }
    // Check, If value of linear X was negative. ( Robot move right )
    else if ((cmdVelLinearX == 0) && (cmdVelAngularZ > 0)) {
      setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 100);
      setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, -100);
    }
    // If there was't any match condition one of above.
    else {
      setMotorSpeed(DIRpinMotorLeft, PWMpinMotorLeft, 0);
      setMotorSpeed(DIRpinMotorRight, PWMpinMotorRight, 0);
    }
  }
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
    "micro_ros_status_message"));

  // TUTA DEBUGGGGGGGGGGGGGGGGg
  RCCHECK(rclc_publisher_init_default(
    &publisherDebug,                                   // Publisher error object
    &node,                                             // Node that owns the publisher
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),  // Message type
    "DEBUG_topic"));                                   // Topic name

  // Create timer callback.
  const unsigned int timer_timeout = 100;  // timer period in milliseconds (1000 ms = 1 second)
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
}


// ---------- Main ----------

void setup() {
  // put your setup code here, to run once:

  // Call the function to setup micro-ROS
  microROSSetup();
  encoderSetup();
  motorAndLedcSetup();
  basicSetup();
}

void loop() {
  // put your main code here, to run repeatedly:

  // Spin the node to run subscriber Twist repeatedly
  RCCHECK(rclc_executor_spin_some(
    &executor_subscriber_Twist,  // Executor subscriber object
    RCL_MS_TO_NS(100)));   // Allow excutor to run 10 hz

  // Spin the node to run subscriber Int8 repeatedly
  RCCHECK(rclc_executor_spin_some(
    &executor_subscriber_Int8,  // Executor subscriber object
    RCL_MS_TO_NS(100)));   // Allow excutor to run 10 hz

  // Spin the node to run publisher repeatedly
  RCCHECK(rclc_executor_spin_some(
    &executor_publisher,  // Executro publisher object
    RCL_MS_TO_NS(100)));  // Allow excutor to run 10 hz
}
