#include <ESP32Encoder.h>
#include <Arduino.h>


// Pin usage 1 motor => (32 = encoderD3MotorLeft), (33 = encoderD2MotorLeft), (25 = PWM), (26 = Dir)
#define PWMpinMotorLeft        25
#define DIRpinMotorLeft        26
#define encoderD3MotorLeft     32
#define encoderD2MotorLeft     33


// Declare class for encoder
ESP32Encoder encoderLeft;


// Declare the setup motor drive board
const int pwmFrequency = 20000;          // Quite motor with more frequency in one duty cycle
const int pwmResolution = 8;


// Create function limit value
int limitValue( int inputValue, int minValue, int maxValue ){
    int outputValue = 0;
    if( inputValue <= minValue ){
        outputValue = minValue;
    }
    else if( inputValue >= maxValue ){
        outputValue = maxValue;
    }
    else{
        outputValue = inputValue;
    }
    return outputValue;
}


// Function for control speed of motor
void setMotorSpeed( int DIRPinInput, int PWMPinInput, int speedMotor ){
    int limitSpeed = limitValue( speedMotor, -255, 255 );
    
    // if value is posible => motor forward
    if( limitSpeed > 0 ){
        // Motor forward
        digitalWrite( DIRPinInput, 1 );
        // Create pulse width modulation function
        ledcWrite( PWMPinInput, limitSpeed );
    }
    
    // if value is negative => motor reverse
    else if( limitSpeed < 0 ){
        // Motor reverse
        digitalWrite( DIRPinInput, 0 );
        // Create pulse width modulation function
        ledcWrite( PWMPinInput, -limitSpeed );
    }
    // if value is zero => stop motor
    else if( limitSpeed == 0 ){
        digitalWrite( DIRPinInput, 1 );
        ledcWrite( PWMPinInput, limitSpeed );
    }
}


// Create output function for visualize the encoder position
void displayCount(){
	// Print serial count
	Serial.println( "Encoder count motor left: " + String( encoderLeft.getCount() ) );
    Serial.println( "Encoder count motor right: " + String( encoderRight.getCount() ) );
}


// Create counter function similar to delay function which visualize the count of encoder
void displayPositionEvery( int valueTimer ){
  //Time function
  unsigned long currentMillis = millis();
	unsigned long previousMillis = 0;
	if( currentMillis - previousMillis >= valueTimer ){
		previousMillis = currentMillis;
		
		// Visualize the encoder count
		displayCount();
	}

}


// Declare the PID config
double kp, ki, kd;
double targetPoint;
double dt, lastTime;
double integral, previous; 
double outputLeft = 0; 


// Create function: motor control using PID. 
double PIDPositionControl( double error ){
    double proportional = error;
    integral += error * dt;
    double derivative = ( error - previous ) / dt;
    previous = error;
    double output = ( kp * proportional ) + ( ki * integral ) + ( kd * derivative );
    return output;
}


// Create counter function similar to delay function which using the pointer to another function
void similarDelay( int valueTimer, void (*func)( int )( int )( int ), int inputValue1, int inputValue2, int inputValue3 ){	
    // Time fucntion
	unsigned long currentMillis = millis();
	unsigned long previousMillis = 0;
	if( currentMillis - previousMillis >= valueTimer ){
		previousMillis = currentMillis;
		
		// Call function which using pointer
		func( inputValue1, inputValue2, inputValue3 );
	}
    
}

// Global variable
long prevT = 0;
int posPrev = 0;



void setup() {
    Serial.begin(115200);

    // Setup motor encoder with pullup
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoderLeft.attachFullQuad( encoderD3MotorLeft, encoderD2MotorLeft );
    encoderLeft.clearCount();
    
    // set count value
    encoderLeft.setCount( 0 );

    // Setup Motor( ledcLibrary )
    pinMode( DIRpinMotorLeft, OUTPUT );
    
    //   After esp32 core >= v.3 use this code:
    ledcAttach( PWMpinMotorLeft, pwmFrequency, pwmResolution );

    // Set config PID
    kp = 0.8;
    ki = 0.2;
    kd = 0.001;
    targetPoint = 1200.00;
    lastTime = 0;
}

void loop() {	
    // // Track time stamp
    // double now = millis();
    // // Convert into second
    // dt = ( now - lastTime ) / 1000.00;
    // lastTime = now;

    // // Find the actual position
    // double actualLeft = ( double )encoderLeft.getCount();
    // double errorLeft = targetPoint - actualLeft;
    // outputLeft = PIDPositionControl( errorLeft );

    // // The output need to become the pulse width modulation
    // setMotorSpeed( DIRpinMotorLeft, PWMpinMotorLeft, outputLeft );
    
    // delay( 100 );
    
    
    // Create up-down pulse
    int pwr = 100/3.0*micros()/1.0e6;
    int pos = encoderLeft.count();
    
    // Compute velocity with method 1
    long currT = micros();
    
}
