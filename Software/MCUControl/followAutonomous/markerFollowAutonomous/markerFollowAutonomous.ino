// Libraries to make Vscode Intellisense happy
#include <String.h>
#include <stdio.h>
////////////////////////////    REMOVE ABOVE FOR ARDUINO    ////////////////////////////

void setup() {
	Serial.begin(9600);
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

// Motor control functions (speed: 0-255)
void moveForward(int speed) {

    // Write the brakes LOW
    digitalWrite(leftBrakePin, LOW);
    digitalWrite(rightBrakePin, LOW);

    // Send the speed to the motors 0-255
    analogWrite(leftMotorPin, speed);
    analogWrite(rightMotorPin, speed);
}

void moveRight(int speed) {

    // Write the brakes LOW
    digitalWrite(leftBrakePin, LOW);
    digitalWrite(rightBrakePin, LOW);

    // Send the speed to the motors 0-255
    analogWrite(leftMotorPin, speed);
    analogWrite(rightMotorPin, 0);
}

void moveLeft(int speed) {

    // Write the brakes LOW
    digitalWrite(leftBrakePin, LOW);
    digitalWrite(rightBrakePin, LOW);

    // Send the speed to the motors 0-255
    analogWrite(leftMotorPin, 0);
    analogWrite(rightMotorPin, speed);
}

void moveBackward(int speed) {

    // Write the brakes LOW
    digitalWrite(leftBrakePin, LOW);
    digitalWrite(rightBrakePin, LOW);

    // Send the speed to the motors 0-255
    analogWrite(leftMotorPin, speed);
    analogWrite(rightMotorPin, speed);
}

void moveNeutral() {

    // Write the brakes HIGH
    digitalWrite(leftBrakePin, HIGH);
    digitalWrite(rightBrakePin, HIGH);

    // Send the speed to the motors 0-255
    analogWrite(leftMotorPin, 0);
    analogWrite(rightMotorPin, 0);
}

void moveBrake() {

    // Write the brakes HIGH
    digitalWrite(leftBrakePin, HIGH);
    digitalWrite(rightBrakePin, HIGH);

    // Send the speed to the motors 0-255
    analogWrite(leftMotorPin, 0);
    analogWrite(rightMotorPin, 0);
}

void loop() {
    if (Serial.available()) {

        //////////////////////////SERIAL DATA///////////////////////////
        // Clear BUFFERs
        memset(BUFFER, 0, sizeof(BUFFER));
        memset(CLEANOUT, 0, sizeof(CLEANOUT));

        // Read the input Serial Data over UART and store the size of the
        size_t readBytes = Serial.readBytesUntil("U", BUFFER, sizeof(BUFFER) - 1);

        // Iterate through the BUFFER string and omit bad data
        for (int i = 0; i < sizeof(BUFFER) - 1; i++) {

            // Only add the data that contains integers and commas
            if (BUFFER[i] != "\n" && BUFFER[i] != "U" ) {

                CLEANOUT[cleanindex] = BUFFER[i];
                cleanindex++;
                isFailure = 0;
            }
            else {

                // Clear the buffers and trigger failure
                memset(BUFFER, 0, sizeof(BUFFER));
                memset(CLEANOUT, 0, sizeof(CLEANOUT));
                isFailure = 1;
                break;
            }

            // If the data is good, convert the data to integers
            if (isFailure == 0) {
                sscanf(CLEANOUT, "%d,%d,%d", &x_center, &y_center, &marker_distance);
                markerPosition[0] = x_center;
                markerPosition[1] = y_center;
                markerPosition[2] = marker_distance;
            }
        }

        ////////////////////////////////////////////////////////////////


        //////////////////// CONTROL CODE /////////////////////////
        // TODO: Make a grade slope speed adjustment based on the y-axis of the marker
        // TODO: Add kill switch mechanism somehow

        // If the marker is over 30 units above the follow distance, move forward
        if (followDistance - markerPosition[2] > 20) {
            moveForward(255);
            Serial.println("Moving Forward");
        }

        // If the marker is between 20 and -20 units of the follow distance, coast
        if (followDistance - markerPosition[2] <= 20 && followDistance - markerPosition[2] >= -20) {
            moveNeutral();
            Serial.println("Coasting");
        }

        // If the marker is between 20 and 60 units below the follow distance, move backward
        if (followDistance - markerPosition[2] < -20 && followDistance - markerPosition[2] > -60) {
            moveBackward(150);
            Serial.println("Moving Backward Slowly");
        }

        // If the marker is over 60 units below the follow distance, move backward
        if (followDistance - markerPosition[2] <= -60) {
            moveBackward(255);
            Serial.println("Moving Backward");
        }

        // Create a bounding range of 840-1080 for the marker to be in the center
        if (markerPosition[0] > 1080) {
            moveRight(255);
            Serial.println("Moving Right");
        }
        if (markerPosition[0] < 840) {
            moveLeft(255);
            Serial.println("Moving Left");
        }
        if (markerPosition[0] >= 840 && markerPosition[0] <= 1080) {
            moveNeutral();
            Serial.println("Coasting");
        }
      /////////////////////////////////////////////////////////
    }

}