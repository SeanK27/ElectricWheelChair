const int joyYPin = A0;
const int joyXPin = A1;

const int leftBrakePin = 8;
const int rightBrakePin = 7;

const int leftReversePin = 13;
const int rightReversePin = 12;

const int rightMotorPin = 5;
const int leftMotorPin = 6;

const int buttonPin = 2;


void setup() {
  Serial.begin(9600);

  // Joystick values
  pinMode(joyXPin, INPUT);
  pinMode(joyYPin, INPUT);


  // Brake Pins
  pinMode(leftBrakePin, OUTPUT);
  pinMode(rightBrakePin, OUTPUT);


  // Reverse Pins
  pinMode(leftReversePin, INPUT);
  pinMode(rightReversePin, INPUT);


  // Motor Pins
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);

  // Button Pin
  pinMode(buttonPin, INPUT_PULLUP);


  unreverseLeft();
  unreverseRight();
}



const int RESTING_X_RAW = 514;
const int RESTING_Y_RAW = 512;
double raw_x, raw_y;
double x_pos, y_pos;

// Sensitivity is how many values (on 1023 scale) to ignore
// when determining things like if the stick is pressed forward or not
// (If stick is between resting + or - sensitivity, do nothing.)
const int SENSITIVITY_X = 5;
const int SENSITIVITY_Y = 10;
const int SENSITIVITY_BRAKE = 200;

double turn_power = 0;
char turning_direction = 's';
bool strong_turn = false;

double forward_power = 0;
bool braking = false;

bool agility_mode = false;

const int MAX_AGILITY_TURNING_SPEED = 255;
const int MAX_AGILITY_TRANSLATIONAL_SPEED = 191;
const int AGILITY_ACCELERATION_TIME = 10;

// Main loop
void loop() {

  // Read raw x and y values
  raw_x = analogRead(joyXPin);
  raw_y = analogRead(joyYPin);

  // Sets x and y positions to 0, 0 when the stick is not pressed
  x_pos = raw_x - RESTING_X_RAW;
  y_pos = raw_y - RESTING_Y_RAW;

  if (digitalRead(buttonPin) == LOW){
    digitalWrite(rightBrakePin, HIGH);
    digitalWrite(leftBrakePin, HIGH);
    delay(500);
    digitalWrite(rightBrakePin, LOW);
    digitalWrite(leftBrakePin, LOW);
    
    if (agility_mode == false){
        agility_mode = true;
    }
    else{
        agility_mode = false;
        unreverseLeft();
        unreverseRight();
    }
  }


  if (agility_mode == false){
    // We don't want strong turn or braking to accidentally be true
    strong_turn = false;
    braking = false;

    // If the stick is pressed forward,
    if (raw_y > RESTING_Y_RAW){
        calculateForwardValuesSpeedMode();
    }

    // If the stick is pressed back (use sensitivity here since back is a stronger turn),
    else if (raw_y <= (RESTING_Y_RAW - SENSITIVITY_Y)){
        calculateBackwardValuesSpeedMode();
    }

    // Else we're not past the threshold
    else{
        turning_direction = 's';
        forward_power = 0;
    }


    // Actually push the motors
    pushMotorsSpeedMode(false);
  }


  // If agility mode is enabled,
  else{
    // If the stick is pressed forward,
    if (raw_y >= 1023){
      // If the stick is forward in the center, move forward slowly
      if (raw_x >= 255 && raw_x <= 769){
        unreverseLeft();
        unreverseRight();
        pushMotorsForwardAgilityMode();
      }
      // If the stick is forward to the left,
      else if (raw_x < 255){
        unreverseLeft();
        unreverseRight();
        strongLeftAgilityMode();
      }
      // If the stick is forward to the right,
      else if (raw_x > 769){
        unreverseLeft();
        unreverseRight();
        strongRightAgilityMode();
      }
    }
    // If the stick is pressed to the right side,
    else if (raw_y < 1023 && raw_y > 0 && raw_x == 1023){
      turnRightOnSpot();
    }
    // If the stick is pressed to the left side,
    else if (raw_y < 1023 && raw_y > 0 && raw_x == 0){
      turnLeftOnSpot();
    }
    // If the stick is pressed down,
    else if (raw_y <= 0){
        // If the stick is down and in the center,
        if (raw_x >= 255 && raw_x <= 769){
          pushMotorsBackwardAgilityMode();
        }
        // If the stick is down and right,
        else if (raw_x > 769){
          strongBackLeftAgilityMode();
        }
        // If the stick is down and left,
        else if (raw_x < 255){
          strongBackRightAgilityMode();
        }
    }
    else{
      noPowerMotors();
    }
  }
}


void noPowerMotors(){
  analogWrite(rightMotorPin, 0);
  analogWrite(leftMotorPin, 0);
}


void reverseRight(){
  pinMode(rightReversePin, INPUT);
}


void reverseLeft(){
  pinMode(leftReversePin, OUTPUT);
  digitalWrite(leftReversePin, LOW);
}


void unreverseRight(){
  pinMode(rightReversePin, OUTPUT);
  digitalWrite(rightReversePin, LOW);
}


void unreverseLeft(){
  pinMode(leftReversePin, INPUT);
}


void accelerateBackwardAgility(){
  reverseLeft();
  reverseRight();
  analogWrite(rightMotorPin, 255);
  analogWrite(leftMotorPin, 255);
  delay(AGILITY_ACCELERATION_TIME);
  analogWrite(rightMotorPin, 0);
  analogWrite(leftMotorPin, 0);
}


void strongBackRightAgilityMode(){
  accelerateBackwardAgility();
  
  digitalWrite(rightBrakePin, LOW);
  digitalWrite(leftBrakePin, HIGH);

  // We don't need this because of above
  unreverseLeft();
  reverseRight();

  analogWrite(rightMotorPin, MAX_AGILITY_TURNING_SPEED);
}


void strongBackLeftAgilityMode(){
  accelerateBackwardAgility();
  
  digitalWrite(rightBrakePin, HIGH);
  digitalWrite(leftBrakePin, LOW);

  unreverseRight();
  reverseLeft();

  analogWrite(leftMotorPin, MAX_AGILITY_TURNING_SPEED);
}


void pushMotorsBackwardAgilityMode(){
  reverseLeft();
  reverseRight();

  digitalWrite(rightBrakePin, LOW);
  digitalWrite(leftBrakePin, LOW);

  analogWrite(rightMotorPin, MAX_AGILITY_TRANSLATIONAL_SPEED);
  analogWrite(leftMotorPin, MAX_AGILITY_TRANSLATIONAL_SPEED);
}


void turnLeftOnSpot(){
  digitalWrite(leftBrakePin, LOW);
  digitalWrite(rightBrakePin, LOW);

  unreverseRight();
  reverseLeft();

  analogWrite(rightMotorPin, MAX_AGILITY_TURNING_SPEED);
  analogWrite(leftMotorPin, MAX_AGILITY_TURNING_SPEED);
}


void turnRightOnSpot(){
  digitalWrite(leftBrakePin, LOW);
  digitalWrite(rightBrakePin, LOW);

  unreverseLeft();
  reverseRight();

  analogWrite(rightMotorPin, MAX_AGILITY_TURNING_SPEED);
  analogWrite(leftMotorPin, MAX_AGILITY_TURNING_SPEED);
}


void strongRightAgilityMode(){
    digitalWrite(leftBrakePin, LOW);
    digitalWrite(rightBrakePin, HIGH);

    analogWrite(leftMotorPin, MAX_AGILITY_TURNING_SPEED);
}


void strongLeftAgilityMode(){
  digitalWrite(leftBrakePin, HIGH);
  digitalWrite(rightBrakePin, LOW);

  analogWrite(rightMotorPin, MAX_AGILITY_TURNING_SPEED);
}



void pushMotorsForwardAgilityMode(){
  digitalWrite(leftBrakePin, LOW);
  digitalWrite(rightBrakePin, LOW);

  analogWrite(leftMotorPin, MAX_AGILITY_TRANSLATIONAL_SPEED);
  analogWrite(rightMotorPin, MAX_AGILITY_TRANSLATIONAL_SPEED);
}



void calculateBackwardValuesSpeedMode(){
  if (raw_x <= RESTING_X_RAW - SENSITIVITY_BRAKE){
    strong_turn = true;
    turning_direction = 'l';
  }

  else if (raw_x >= (RESTING_X_RAW + SENSITIVITY_BRAKE)){
    strong_turn = true;
    turning_direction = 'r';
  }

  else{
    braking = true;
  }
}


void calculateForwardValuesSpeedMode(){
  // All these turn power values are changed because of a UI change
  
  turn_power = 0;
  // If trying to turn right,
  if (raw_x > (RESTING_X_RAW + SENSITIVITY_X)){
    turning_direction = 'r';
    // Max turning is 50% when x = 1023 and y = 1023.
    // As y starts to decrease (until resting + sensitivity), 
    // gradually increase turning to 100%
    if (raw_x < 1023){
      turn_power = 1 * ((x_pos)/(1023 - RESTING_X_RAW));
    }

    // x >= 1023, meaning stick is at or beyond quadrant 1 dead area
    else{
      turn_power = 1;
      //turn_power = 0.5 + (0.5*((RESTING_Y_RAW - (raw_y - 511))/(RESTING_Y_RAW)));
    }
  }

  // If trying to turn left,
  else if (raw_x < (RESTING_X_RAW - SENSITIVITY_X)){
    turning_direction = 'l';
    // Before dead zone
    if (raw_x > 0){
      turn_power = 1*(((1023 - RESTING_X_RAW) - raw_x)/(1023 - RESTING_X_RAW));
    }
    // After dead zone (x <= 0)
    else{
      turn_power = 1;
    //   turn_power = 0.5 + (0.5*((RESTING_Y_RAW - (raw_y - 511))/(RESTING_Y_RAW)));
    }
  }

  // Else we're not past the turning threshold
  else{
    // 's' for "straight"
    turning_direction = 's';
    turn_power = 0;
    forward_power = ((raw_y - RESTING_Y_RAW)/(1023 - RESTING_Y_RAW));
  }
}


void pushMotorsSpeedMode(bool debug){
  if (braking == true){
    if (debug == true){
      Serial.println("Braking...");
    }
    digitalWrite(leftBrakePin, HIGH);
    digitalWrite(rightBrakePin, HIGH);
  }
  else if (turning_direction == 'r'){
    if (strong_turn == false){
        if (debug == true){
          Serial.print("Weak right turn with power ");
          Serial.print(turn_power);
          Serial.println();
        }

      digitalWrite(leftBrakePin, LOW);
      digitalWrite(rightBrakePin, LOW);
      analogWrite(leftMotorPin, 255);
      analogWrite(rightMotorPin, (255 * (1 - turn_power)));
    }
    else{
      if (debug == true){
        Serial.println("Strong right turn");
      }
      digitalWrite(leftBrakePin, LOW);
      analogWrite(leftMotorPin, 255);
      digitalWrite(rightBrakePin, HIGH);
    }
  }
  else if (turning_direction == 'l'){
    if (strong_turn == false){
      if (debug == true){
        Serial.print("Weak left turn with power ");
        Serial.print(turn_power);
        Serial.println();
      }

      digitalWrite(leftBrakePin, LOW);
      digitalWrite(rightBrakePin, LOW);
      analogWrite(leftMotorPin, (255 * (1 - turn_power)));
      analogWrite(rightMotorPin, 255);
    }
    else{
      if (debug == true){
        Serial.println("Strong left turn");
      }
      digitalWrite(rightBrakePin, LOW);
      analogWrite(rightMotorPin, 255);
      digitalWrite(leftBrakePin, HIGH);
    }
  }
  else{
    if (debug == true){
      Serial.print("Moving forward with power ");
      Serial.print(forward_power);
      Serial.println();
    }

    digitalWrite(leftBrakePin, LOW);
    digitalWrite(rightBrakePin, LOW);
    analogWrite(leftMotorPin, (255 * forward_power));
    analogWrite(rightMotorPin, (255 * forward_power));
  }
}
