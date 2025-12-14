import cv2
import cv2.aruco as aruco
import os

# Create a directory for textures if it doesn't exist
texture_path = "../models/textures"
os.makedirs(texture_path, exist_ok=True)

# Define dictionary (using Original ArUco for simplicity)
dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

for i in range(5):
    # Generate marker: ID=i, Size=200px
    img = aruco.drawMarker(dictionary, i, 200) 
    
    filename = f"{texture_path}/marker_{i}.png"
    cv2.imwrite(filename, img)
    print(f"Generated {filename}")