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

// unsigned long currentMillis = millis();
// unsigned long previousMillis = 0;

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
        digitalWrite( PWMpin, 1 );
        ledcWrite( PWMpin, limitSpeed );
    }
}

void displayCount(){
	// Print serial count
	Serial.println( "Encoder count: " + String( encoder.getCount() ) );
}

// Create counter function similar to delay function which visualize the count of encoder
void displayAndDelay( int valueTimer ){
    //Time function
    unsigned long currentMillis = millis();
	unsigned long previousMillis = 0;
	if( currentMillis - previousMillis >= valueTimer ){
		previousMillis = currentMillis;
		
		// Visualize the encoder count
		displayCount();
	}

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
  encoder.setCount( 666 );

  // Setup Motor( ledcLibrary )
  pinMode(DIRpin, OUTPUT);
  //   After esp32 core >= v.3 use this code:
  ledcAttach(PWMpin, pwmFrequency, pwmResolution);
}

void loop() {	
	// Call function timer and set speed motor
    displayAndDelay( 2000 );
    // for( int iteration = 0; iteration <= 5; iteration++ ){
    //     similarDelay( 5000, setMotorSpeed, 100 );
    //     similarDelay( 10000, setMotorSpeed, 0 );
    //     similarDelay( 15000, setMotorSpeed, -100 );
    //     similarDelay( 20000, setMotorSpeed, 0 );
    //     // previousMillis = currentMillis;   
    // }
    similarDelay( 5000, setMotorSpeed, 100 );
    similarDelay( 10000, setMotorSpeed, 0 );
    similarDelay( 15000, setMotorSpeed, -100 );
    similarDelay( 20000, setMotorSpeed, 0 );
}
