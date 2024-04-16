# -----------------------------------------------------------------------------------
# Correct an image taken underwater with the correction maps generated by the Pinax Model. 

# Author: Adrienne Winter under African Robotics Unit, University of Cape Town, 2022.

# README:
# This script assumes that the Pinax Model MatLab code "FindMap.m" has already been run
# and the files MapX.txt and MapY.txt exist. It saves corrected images in the same folder
# as the original image. If you are using stereo, make sure you are using the correct Pinax 
# maps with the correct images (left or right). The Pinax model removes lens and refraction 
# distortion, but does not stereo rectify the images. You will need to stereo rectify with 
# distortion coefficients as zero after the Pinax undistortion step.

# You need to set the variables imgName, imgPath, mapPath and change the map.txt names accordingly
# -----------------------------------------------------------------------------------

# import libraries
import cv2 as cv
import numpy as np

# set variables
imgName = "cam0_distorted.png"
imgPath = "path\\to\\image\\directory"
mapPath = "path\\to\\Pinax\\map\\directory"

# import image
img = cv.imread(f"{imgPath}\\{imgName}")

# import the MatLab generated maps
map_x = np.loadtxt(f"{mapPath}\\Left_MapX.txt",dtype=np.float32,delimiter=",")
map_y = np.loadtxt(f"{mapPath}\\Left_MapY.txt",dtype=np.float32,delimiter=",")

# correct image and save
correctedImg = cv.remap(img, map_x, map_y, cv.INTER_LINEAR)
imgName = imgName.split(".")[0]
cv.imwrite(f"{imgPath}\\{imgName}_PinaxUndistorted.jpg", correctedImg)

print("Image corrected.")