const int joyXPin = A0;
const int joyYPin = A1;

const int leftBrakePin = 8;
const int rightBrakePin = 7;

const int leftReversePin = 13;
const int rightReversePin = 12;

const int rightMotorPin = 5;
const int leftMotorPin = 6;

void setup() {
  Serial.begin(9600);

  // Joystick values
  pinMode(joyXPin, INPUT);
  pinMode(joyYPin, INPUT);


  // Brake Pins
  pinMode(leftBrakePin, OUTPUT);
  pinMode(rightBrakePin, OUTPUT);


  // Reverse Pins
  pinMode(leftReversePin, OUTPUT);
  pinMode(rightReversePin, OUTPUT);


  // Motor Pins
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
}



// Define position array [x,y]
int xyData[2];

// Random parameter that affects turning sensitivity
double turnFactor = 1;

// Random variables for speed and calcs
double base, adjust, rightMotor, leftMotor;



// Main loop
void loop() {

  // Store current joystick position to array
  xyData[0] = analogRead(joyXPin);
  xyData[1] = analogRead(joyYPin);

  // Initialize base variable by mapping the current vertical joystick value to 0-255 scale.
  // Base variable is the unadjusted speed. Ex. joystick halfway forward would be 127.5 base value.
  base = map(xyData[0], 512, 1024, 0, 255);

  // Adjustment value that is applied to both motors
  adjust = (map(512 - xyData[1], 0, 1024, 0, 255)) * turnFactor;

  // Apply adjustment
  rightMotor = base - adjust;
  leftMotor = base + adjust;

  //Serial.print("Left Motor: ");
  //Serial.println(leftMotor);
  //Serial.print("Right Motor: ");
  //Serial.println(rightMotor);

  // Braking implementation. ACTIVE HIGH.
  // When joystick is fully back, send HIGH signal to brakePinLeft then set power to 0.
  if (rightMotor < -240 && leftMotor < -240) {
    digitalWrite(rightBrakePin, HIGH);
    digitalWrite(leftBrakePin, HIGH);
    rightMotor = 0;
    leftMotor = 0;
    Serial.println("BRAKE");
  }

  
  else {

    // Remove any braking before sending power
    digitalWrite(rightBrakePin, LOW);
    digitalWrite(leftBrakePin, LOW);
    Serial.println("NO BRAKE");


    //////////////// Removing values >255 or <0 and setting them to 255 and 0 respectively ///////////////////////
    if (rightMotor < 0) {
      rightMotor = 0;
    }
  
    if (leftMotor < 0) {
      leftMotor = 0;
    }
  
    if (rightMotor > 255) {
      rightMotor = 255;
    }
  
    if (leftMotor > 255) {
      leftMotor = 255;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////


  
    // Send output PWM signal to leftMotor and rightMotor (0-255)
    analogWrite(leftMotorPin, leftMotor);
    

    //rightMotor
    analogWrite(rightMotorPin, rightMotor); 
  }
}
