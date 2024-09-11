void setup() {
    Serial.begin(9600);
}

// Define marker position and distance array [x, y, distance]
int markerPosition[3] = {0, 0, 0};

// Define buffer for raw serial data
char BUFFER[32];

void loop() {
    if (Serial.availible()) {
      
      // Read the input Serial Data over UART and store the size of the 
      size_t readBytes = Serial.readBytesUntil("\n", BUFFER, sizeof(BUFFER) - 1);
      BUFFER[readBytes] = "\0";

      // Process the input data
      sscanf(BUFFER, "%d, %d, %d", &x_center, &y_center, &marker_distance);

      // Print the received values
      Serial.print("Received: ");
      Serial.print("x_center = ");
      Serial.print(x_center);
      Serial.print(", y_center = ");
      Serial.print(y_center);
      Serial.print(", marker_distance = ");
      Serial.println(marker_distance);

    }
}