#include <ESP32Encoder.h>
#include <Arduino.h>

// Pin usage => (32 = encoderD3), (33 = encoderD2), (25 = PWM), (26 = Dir)
#define PWMpin          25
#define DIRpin          26
#define encoderD3       32
#define encoderD2       33

// Declare class for encoder
ESP32Encoder encoder;

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
void setMotorSpeed( int speedMotor ){
    int limitSpeed = limitValue( speedMotor, -255, 255 );
    
    // if value is posible => motor forward
    if( limitSpeed > 0 ){
        // Motor forward
        digitalWrite( DIRpin, 1 );
        // Create pulse width modulation function
        ledcWrite( PWMpin, limitSpeed );
    }
    
    // if value is negative => motor reverse
    else if( limitSpeed < 0 ){
        // Motor reverse
        digitalWrite( DIRpin, 0 );
        // Create pulse width modulation function
        ledcWrite( PWMpin, -limitSpeed );
    }
    // if value is zero => stop motor
    else if( limitSpeed == 0 ){
        digitalWrite( DIRpin, 1 );
        ledcWrite( PWMpin, limitSpeed );
    }
}

void displayCount(){
	// Print serial count
	Serial.println( "Encoder count: " + String( encoder.getCount() ) );
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
double integral, previous, output = 0; 

// Create function: motor control using PID. 
double PIDControl( double error ){
    double proportional = error;
    integral += error * dt;
    double derivative = ( error - previous ) / dt;
    previous = error;
    double output = ( kp * proportional ) + ( ki * integral ) + ( kd * derivative );
    return output;
}

void PIDControlAutoTune( int targetPos ){
    // This function was using differential and integral to auto tune PID value
}

// Create counter function similar to delay function which using the pointer to another function
void similarDelay( int valueTimer, void (*func)(int), int inputValueFunc ){
	// Time fucntion
	unsigned long currentMillis = millis();
	unsigned long previousMillis = 0;
	if( currentMillis - previousMillis >= valueTimer ){
		previousMillis = currentMillis;
		
		// Call function which using pointer
		func( inputValueFunc );
	}
    
}

void setup() {
    Serial.begin(115200);

    // Setup motor encoder with pullup
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachFullQuad( encoderD3, encoderD2 );
    encoder.clearCount();
    // set count value
    encoder.setCount( 0 );

    // Setup Motor( ledcLibrary )
    pinMode(DIRpin, OUTPUT);
    //   After esp32 core >= v.3 use this code:
    ledcAttach(PWMpin, pwmFrequency, pwmResolution);

    // Set config PID
    kp = 0.8;
    ki = 0.2;
    kd = 0.001;
    targetPoint = 1200.00;
    lastTime = 0;
}

void loop() {	
    // Call function timer and set speed motor
    // similarDelay( 5000, setMotorSpeed, 100 );
    // similarDelay( 10000, setMotorSpeed, 0 );
    // similarDelay( 15000, setMotorSpeed, -100 );
    // similarDelay( 20000, setMotorSpeed, 0 );
    // Call function visualizatio position
    // displayPositionEvery( 2000 );

    // Track time stamp
    double now = millis();
    // Convert into second
    dt = ( now - lastTime ) / 1000.00;
    lastTime = now;

    // Find the actual position
    double actual = ( double )encoder.getCount();
    double error = targetPoint - actual;
    output = PIDControl( error );

    // The output need to become the pulse width modulation
    setMotorSpeed( output );

    // Visualization
    Serial.print( targetPoint );
    Serial.print( "," );
    Serial.println( actual );

    delay( 300 );

}
