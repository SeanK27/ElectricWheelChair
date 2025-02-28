import cv2 as cv
import serial
import time

def loopback_test(ser):
    test_message = "test message\n"
    ser.write(test_message.encode())
    time.sleep(1)  # Wait for the message to be sent and received
    if ser.in_waiting > 0:
        received_message = ser.read(ser.in_waiting).decode()
        if received_message == test_message:
            print("Loopback test successful")
        else:
            print("Loopback test failed")
    else:
        print("Loopback test failed")


# Simple XOR checksum
def calculate_checksum(data):
    
    checksum = 0
    for char in data:
        checksum ^= ord(char)  # XOR each character's ASCII value
    return checksum

# Send formatted data
def send_serial_data(ser, x, y, distance):

    data = f"{x},{y},{distance}"
    checksum = calculate_checksum(data)
    message = f"{data},{checksum}U"
    ser.write(message.encode())

# Open a serial connection to the Arduino
ser = serial.Serial('COM5', 9600)

# Perform loopback test
# loopback_test(ser)

# Load the camera
cap = cv.VideoCapture(0)

# Define the ArUco dictionary and parameters
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
parameters = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

while True:
    # Capture each frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        break
    
    # Convert frame to grayscale (required for ArUco detection)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Detect the ArUco markers in the grayscale image
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)

    # If at least one marker is detected
    if markerIds is not None:
        # Draw bounding box around detected markers
        cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

        # Get the coordinates of the marker's center and size
        for corners, markerId in zip(markerCorners, markerIds):
            # Get the center of the marker
            x_center = int((corners[0][0][0] + corners[0][2][0]) / 2)
            y_center = int((corners[0][0][1] + corners[0][2][1]) / 2)
            print(f"Marker ID: {markerId[0]}, Center: ({x_center}, {y_center})")

            """
            # Calculate the size of the marker (distance between two opposite corners)
            marker_distance = int(cv.norm(corners[0][0] - corners[0][2]))
            print(f"Marker ID: {markerId[0]}, Size: {marker_distance}")
            """

            # Calculate the size of the marker using the average distance between all pairs of opposite corners
            size1 = cv.norm(corners[0][0] - corners[0][2])
            size2 = cv.norm(corners[0][1] - corners[0][3])
            marker_distance = int((size1 + size2) / 2)
            print(f"Marker ID: {markerId[0]}, Size: {marker_distance}")

            ### TODO: Make a better calculation for the marker size and use some calc or something ###

            # Send the coordinates and size over serial
            send_serial_data(ser, x_center, y_center, marker_distance)

            # Draw a circle at the calculated center
            cv.circle(frame, (x_center, y_center), 5, (0, 255, 0), -1)

    # Output the frame with detection boxes and centers displayed
    cv.imshow('ArUco Marker Detection', frame)

    # Break the loop if 'q' is pressed
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv.destroyAllWindows()
ser.close()