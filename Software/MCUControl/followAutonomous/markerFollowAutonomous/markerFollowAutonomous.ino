// Libraries to make Vscode Intellisense happy
#include <String.h>
#include <stdio.h>
#include <PID_v1.h>
////////////////////////////    REMOVE ABOVE FOR ARDUINO    ////////////////////////////

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

// Speeds
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// Dropout settings
unsigned long lastValidDataTime = 0;        // Time when last valid data was received
const unsigned long dropoutTimeout = 1000;  // Alive timeout
bool isDropout = false;

// Distance to maintain from the marker
const int followDistance = 186;

// Center alignment for yaw
const int centerX = 960;

// PID control stuff for distance
double KpD = 1.2, KiD = 0, KdD = 0;       /////// TODO: TUNE THESE
double setpointDistance = followDistance, inputDistance, outputDistance;

// PID control stuff for yaw
double KpY = .15, KiY = 0, KdY = 0;       /////// TODO: TUNE THESE
double setpointYaw = centerX, inputYaw, outputYaw;

PID distancePID(&inputDistance, &outputDistance, &setpointDistance, KpD, KiD, KdD, DIRECT);

PID yawPID(&inputYaw, &outputYaw, &setpointYaw, KpY, KiY, KdY, DIRECT);

void setup() {
	
    Serial.begin(115200);

    yawPID.SetMode(AUTOMATIC);
    yawPID.SetOutputLimits(-255, 255);

    distancePID.SetMode(AUTOMATIC);

    // Initialize hardware pins
    pinMode(leftBrakePin, OUTPUT);
    pinMode(rightBrakePin, OUTPUT);
    pinMode(leftReversePin, OUTPUT);
    pinMode(rightReversePin, OUTPUT);
    pinMode(rightMotorPin, OUTPUT);
    pinMode(leftMotorPin, OUTPUT);
    pinMode(buttonPin, INPUT);
}

// Define marker position and distance array [x, y, distance]
int markerPosition[3] = { 0, 0, 0 };

// Define buffer for raw serial data
char BUFFER[14];
int isFailure;

// Define position variables
int x_center, y_center, marker_distance;

// Motor control functions (speed: 0-255). Brakes cut when moving/coasting
void move(int leftSpeed, int rightSpeed) {
    digitalWrite(leftBrakePin, LOW);
    digitalWrite(rightBrakePin, LOW);
    analogWrite(leftMotorPin, constrain(leftSpeed, 0, 255));
    analogWrite(rightMotorPin, constrain(rightSpeed, 0, 255));
    if (leftSpeed == 0) digitalWrite(leftBrakePin, HIGH);
    if (rightSpeed == 0) digitalWrite(rightBrakePin, HIGH);
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
    return checksum & 0xFF;  // Lower byte of checksum is sent
}

void loop() {
    if (Serial.available()) {

        //////////////////////////SERIAL DATA///////////////////////////
        // Clear BUFFERs
        memset(BUFFER, 0, sizeof(BUFFER));

        // Read the input Serial Data over UART and store the size of the
        size_t readBytes = Serial.readBytesUntil('U', BUFFER, sizeof(BUFFER));
        int receivedChecksum;

        // Input Integrity Checks
        if (sscanf(BUFFER, "%d,%d,%d,%d", &x_center, &y_center, &marker_distance, &receivedChecksum) == 4) {

            // Find the last comma (before the checksum)
            char *lastComma = strrchr(BUFFER, ',');
            if (lastComma) {
                *lastComma = '\0';  // Truncate string at last comma
            }

            int computedChecksum = calculateChecksum(BUFFER);
            
            // Store data if checksum valid
            if (computedChecksum == receivedChecksum) {
                
                // Check for dropout (0,0,0)
                if (x_center == 0 && y_center == 0 && marker_distance == 0) {

                    //Serial.println("Dropout detected. Holding last known values.");
                    isDropout = true;

                } else {

                markerPosition[0] = x_center;
                markerPosition[1] = y_center;
                markerPosition[2] = marker_distance;
                lastValidDataTime = millis();
                isDropout = false;
                }
            } else {
                /*
                Serial.print("Checksum Error: Bad Data: ");
                Serial.print(computedChecksum);
                Serial.print(" != ");
                Serial.println(receivedChecksum);
                */
            }
        }

        ////////////////////////////////////////////////////////////////


        //////////////////// CONTROL CODE /////////////////////////
        // TODO: Make a grade slope speed adjustment based on the y-axis of the marker
        // TODO: Add KILL SWITCH, this is pretty important please do this

        // If we are in dropout, check timeout
        if (isDropout && millis() - lastValidDataTime > dropoutTimeout) {
            //Serial.println("Timeout. Stopping motors.");
            moveBrake();
            return;
        }

        // Compute PID for distance
        inputDistance = markerPosition[2];
        distancePID.Compute();
        int baseSpeed = constrain(outputDistance, 0, 255);

        // Compute PID for yaw
        inputYaw = markerPosition[0];
        yawPID.Compute();

        int yawCorrection = constrain(outputYaw, -baseSpeed, baseSpeed);

        // Adjust movement based on PID outs
        leftMotorSpeed = baseSpeed - yawCorrection;
        rightMotorSpeed = baseSpeed + yawCorrection;

        move(leftMotorSpeed, rightMotorSpeed);

        //Serial.println(yawCorrection);
        //Serial.println(leftMotorSpeed);
        Serial.print("Left Motor: ");       Serial.print(leftMotorSpeed);
        Serial.print(" Right Motor: ");     Serial.println(rightMotorSpeed);

        //Serial.print("D_Error:"); Serial.print(setpointDistance - inputDistance);
        //Serial.print(" Y_Error:"); Serial.print(setpointYaw - inputYaw);
        //Serial.print(" PID_Distance:"); Serial.print(outputDistance);
        //Serial.print(" PID_Yaw:"); Serial.println(outputYaw);

      /////////////////////////////////////////////////////////
    }
}