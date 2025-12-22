#include <ESP32Encoder.h>

// Pin usage => (32 = encoderD3), (33 = encoderD2), (25 = PWM), (26 = Dir)
#define PWMpin          25
#define DIRpin          26
#define encoderD3       34
#define encoderD2       35

// Declare class for encoder
ESP32Encoder encoder;

// Declare the setup motor drive board
const int pwmChannel = 0;
const int pwmFreqency = 20000;          // Quite motor with more frequency in one duty cycle
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
    limitSpeed = limitValue( speedMotor, -255, 255 );
    
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

// Create counter function similar to delay function which using the pointer
void similarDelay( int valueTimer, void (*func)(int), int inputValueFunc ){
    // Time fucntion
    unsigned long currentMillis = millis();
    unsigned long previousMillis = 0;
    if( currentMillis - previousMillis >= valueTimer ){
        previousMillis = currentMillis;
        
        // Call function which using pointer
        func( inputValueFunc );
        
        // Print serial count
        Serial.println( "Encoder count: " + string( encoder.getCount() ) )

    }
    
}

void setup() {
  Serial.begin(115200);

  // Setup motor encoder with pullup
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachFullQuad( encoderD3, encoderD2 );
  encoder.clearCount();
  // set count value
  encoder.setCount( 666 );

  // Setup Motor( ledcLibrary )
  pinMode(DIRpin, OUTPUT);
  ledcSetup(pwmChannel, pwmFreqency, pwmResolution);
  ledcAttachPin(PWMpin, pwmChannel);
}

void loop() {
    // Call function timer and set speed motor
    similarDelay( 2000, setMotorSpeed, 100 );
    similarDelay( 2000, setMotorSpeed, 0 );
    similarDelay( 2000, setMotorSpeed, -100 );
}
