void setup() {
  Serial.begin(9600);
}

// Define marker position and distance array [x, y, distance]
int markerPosition[3] = { 0, 0, 0 };

// Define buffer for raw serial data
char BUFFER[13];
int isFailure;

// Cleaned BUFFER for processed serial data
char CLEANOUT[13];
int cleanindex = 0;

// Define position variables
int x_center, y_center, marker_distance;

void loop() {
  if (Serial.available()) {

    // Clear BUFFER
    memset(BUFFER, 0, sizeof(BUFFER));
    memset(CLEANOUT, 0, sizeof(CLEANOUT));

    // Read the input Serial Data over UART and store the size of the
    size_t readBytes = Serial.readBytesUntil("U", BUFFER, sizeof(BUFFER) - 1);

    // Print input data for debugging
    // Serial.println(BUFFER);
    // Serial.println("Endbuffer");

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
    }

    // Print only if data was successfully sent
    if (!isFailure) {
      Serial.println(CLEANOUT);
      Serial.println("Success");
    }
    else {
      Serial.println("Failure");
    }

  }
}




