import cv2 as cv

# Define ArUco dictionary
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)

marker_id = 0  # Marker ID (0-249 for DICT_6X6_250)
marker_size = 2000  # Size of the marker in pixels

# Generate Marker Image
marker_image = cv.aruco.generateImageMarker(dictionary, marker_id, marker_size)

# Save the marker image
cv.imwrite("aruco_marker_ID0.png", marker_image)

# Show marker image
cv.imshow('ArUco Marker', marker_image)
cv.waitKey(0)
cv.destroyAllWindows()

print("ArUco marker successfully saved.")