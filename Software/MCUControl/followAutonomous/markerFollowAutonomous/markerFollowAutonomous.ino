// Libraries to make Vscode Intellisense happy
#include <String.h>
#include <stdio.h>
#include <PID_v1.h>
////////////////////////////    REMOVE ABOVE FOR ARDUINO    ////////////////////////////

void setup() {
	Serial.begin(9600);

    // Initialize hardware pins
    pinMode(leftBrakePin, OUTPUT);
    pinMode(rightBrakePin, OUTPUT);
    pinMode(leftReversePin, OUTPUT);
    pinMode(rightReversePin, OUTPUT);
    pinMode(rightMotorPin, OUTPUT);
    pinMode(leftMotorPin, OUTPUT);
    pinMode(buttonPin, INPUT);
}

// Define and intialize hardware pins
const int joyYPin = A0;
const int joyXPin = A1;

const int leftBrakePin = 8;
const int rightBrakePin = 7;

const int leftReversePin = 13;
const int rightReversePin = 12;

const int rightMotorPin = 5;
const int leftMotorPin = 6;

const int buttonPin = 2;

// Distance to maintain from the marker
const int followDistance = 200;

// Center alignment for yaw
const int centerX = 960;

// Define marker position and distance array [x, y, distance]
int markerPosition[3] = { 0, 0, 0 };

// Define buffer for raw serial data
char BUFFER[14];
int isFailure;

// Cleaned BUFFER for processed serial data
char CLEANOUT[14];
int cleanindex = 0;

// Define position variables
int x_center, y_center, marker_distance;

// PID control stuff for distance
double KpD = 1, KiD = 1, KdD = 1;       /////// TODO: TUNE THESE
double setpointDistance = followDistance, inputDistance, outputDistance;
PID distancePID(&inputDistance, &outputDistance, &setpointDistance, KpD, KiD, KdD, DIRECT);

// PID control stuff for yaw
double KpY = 1, KiY = 1, KdY = 1;       /////// TODO: TUNE THESE
double setpointYaw = centerX, inputYaw, outputYaw;
PID yawPID(&inputYaw, &outputYaw, &setpointYaw, KpY, KiY, KdY, DIRECT);

// Motor control functions (speed: 0-255). Brakes cut when moving/coasting
void move(int leftSpeed, int rightSpeed) {
    digitalWrite(leftBrakePin, LOW);
    digitalWrite(rightBrakePin, LOW);
    analogWrite(leftMotorPin, constrain(leftSpeed, 0, 255));
    analogWrite(rightMotorPin, constrain(rightSpeed, 0, 255));
}

// Cut brakes and cut power
void moveNeutral() {
    digitalWrite(leftBrakePin, LOW);
    digitalWrite(rightBrakePin, LOW);
    analogWrite(leftMotorPin, 0);
    analogWrite(rightMotorPin, 0);
}

// Engage brakes and cut power
void moveBrake() {

    digitalWrite(leftBrakePin, HIGH);
    digitalWrite(rightBrakePin, HIGH);
    analogWrite(leftMotorPin, 0);
    analogWrite(rightMotorPin, 0);
}

// Simple XOR checksum
int calculateChecksum(const char *data) {
    int checksum = 0;
    while (*data) {
        checksum ^= *data++;
    }
    return checksum & 0xFF;  // Lower byte of checksum should be fine
}

void loop() {
    if (Serial.available()) {

        //////////////////////////SERIAL DATA///////////////////////////
        // Clear BUFFERs
        memset(BUFFER, 0, sizeof(BUFFER));

        // Read the input Serial Data over UART and store the size of the
        size_t readBytes = Serial.readBytesUntil('U', BUFFER, sizeof(BUFFER) - 1);
        int receivedChecksum;

        // Input Integrity Checks
        if (sscanf(BUFFER, "%d,%d,%d,%d", &x_center, &y_center, &marker_distance, &receivedChecksum) == 4) {
            
            // Prep and BUFFER for checksum
            BUFFER[readBytes - 1] = '\0';
            int computedChecksum = calculateChecksum(BUFFER);

            // Store data if checksum valid
            if (computedChecksum == receivedChecksum) {
                markerPosition[0] = x_center;
                markerPosition[1] = y_center;
                markerPosition[2] = marker_distance;
                Serial.print("Valid Data: ");
                Serial.print(x_center);
                Serial.print(",");
                Serial.print(y_center);
                Serial.print(",");
                Serial.println(marker_distance);
            } else {
                Serial.println("Checksum Error: Bad Data");
            }
        }

        ////////////////////////////////////////////////////////////////


        //////////////////// CONTROL CODE /////////////////////////
        // TODO: Make a grade slope speed adjustment based on the y-axis of the marker
        // TODO: Add KILL SWITCH, this is pretty important please do this

        // Compute PID for distance
        inputDistance = markerPosition[2];
        distancePID.Compute();
        int motorSpeed = constrain(outputDistance, 0, 255);

        // Compute PID for yaw
        inputYaw = markerPosition[0];
        yawPID.Compute();
        int yawCorrection = constrain(outputYaw, -255, 255);

        // Adjust movement based on PID outs
        int leftMotorSpeed = motorSpeed - yawCorrection;
        int rightMotorSpeed = motorSpeed + yawCorrection;

        move(leftMotorSpeed, rightMotorSpeed);

        Serial.print("Left Motor: ");       Serial.print(leftMotorSpeed);
        Serial.print(" Right Motor: ");     Serial.println(rightMotorSpeed);

      /////////////////////////////////////////////////////////
    }

}