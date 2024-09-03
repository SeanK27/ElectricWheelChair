import cv2 as cv

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

        # Get the coordinates of the marker's center
        for corners, markerId in zip(markerCorners, markerIds):

            # Get the center of the marker
            x_center = int((corners[0][0][0] + corners[0][2][0]) / 2)
            y_center = int((corners[0][0][1] + corners[0][2][1]) / 2)
            print(f"Marker ID: {markerId[0]}, Center: ({x_center}, {y_center})")

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
