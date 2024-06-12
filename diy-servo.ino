#include <PID_v1.h>

#include <Wire.h>
#include <AS5600.h>

#define DEBUG true

const int motorPin1 = 7;  // Digital pin 1 for H-bridge
const int motorPin2 = 9; // Digital pin 2 for H-bridge

// const int motorPin1 = 1;  // Digital pin 1 for H-bridge
// const int motorPin2 = 2; // Digital pin 2 for H-bridge

const unsigned int calcPeriod = 10; //ms between calculations

unsigned long lastCalculation = 100000000;

AMS_5600 ams5600;

double currentAngle;

// PID parameters
double Setpoint, Output;

double Kp = 0.025;
double Ki = 0.2;
double Kd = 0.01;

PID motorPID(&currentAngle, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup(){
    if (DEBUG) {
        Serial.begin(115200);
        while (!Serial) {
            ;  // wait for serial port to connect. Needed for native USB port only
        }
    }
  
    Wire.begin();   // default sda:33 scl:35
    // Wire1.begin(18, 16);

    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);

    controlMotor(0);

    motorPID.SetSampleTime(1);
    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(-255, 255);
    motorPID.SetControllerDirection(DIRECT);

    Setpoint = 2048;

    if (DEBUG) {
        Serial.println("Leaving setup");
    }
}

/*******************************************************
/* Function: convertRawAngleToDegrees
/* In: angle data from AMS_5600::getRawAngle
/* Out: human readable degrees as float
/* Description: takes the raw angle and calculates
/* float value in degrees.
/*******************************************************/
float convertRawAngleToDegrees(word newAngle)
{
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087890625;
  return retVal;
}

void loop()
{
    if(millis() - lastCalculation > calcPeriod) {
        lastCalculation = millis();
        // if (DEBUG) {
        //     Serial.println("reading as5600");
        // }
        if (ams5600.detectMagnet() == true) {
            word tempVal = ams5600.getRawAngle();
            if (tempVal < 4096) {
                currentAngle = tempVal;
            }
        }
        
        if (DEBUG) {
            // Serial.print("Current raw angle: ");
            // Serial.println(currentAngle);
            // Serial.print("Current output: ");
            // Serial.println(Output);
            Serial.print("Current Setpoint: ");
            Serial.println(Setpoint);
        }
        Setpoint++;
        if (Setpoint >= 3500) {
            Setpoint = 1500;
        }
        if (DEBUG) {
            Serial.print("Error: ");
            Serial.println(Setpoint - currentAngle);
        }
    }

    motorPID.Compute();

    // Control the motor based on the PID output
    controlMotor(Output);
    // controlMotor(-255.0);
}

void controlMotor(double output) {
    // Cast the output to an int and constrain it to be between -255 and 255
    int intOutput = (int)output;
    intOutput = constrain(intOutput, -255, 255);

    if (DEBUG) {
    //   Serial.print("output: ");
    //   Serial.println(output);
        // Serial.print("intOutput: ");
        // Serial.println(intOutput); 
    }

    if (intOutput > 0) {
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, intOutput);
    } else if (intOutput < 0) {
        analogWrite(motorPin2, 0);
        analogWrite(motorPin1, -1 * intOutput);
    } else {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
    }
}