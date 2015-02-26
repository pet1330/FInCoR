import cv2
from fpdf import FPDF
import numpy as np
import os

numOfCircles = 3
circleDiameter = 0.05;


pdf = FPDF()
pdf.add_page()
numberPerRow = 0.21 / circleDiameter

print "ROW", numberPerRow

img = np.ones((512, 512, 3), np.uint8)
for i in range(40, numOfCircles + 40):
    img[:, :] = (255, 255, 255)
    for draw_i in range(0, 128-i):
        cv2.circle(img, (256, 256), 256-(draw_i * 2), (0, 153, 255), 2)
    cv2.imwrite("circle.png", img)
    cv2.imshow('image', img)
    k = cv2.waitKey(1000) #& 0xFF
    pdf.image("circle.png", (numOfCircles / 4) * 50, (i / 4), 50, 50, 'png')
    col = i / numberPerRow
    row = i - col * numberPerRow

    print col, row
    
pdf.output('BETA.pdf', 'F')
########################################################################################################################################

#for i in range(0, numberOfCircles):
#    img[:, :] = (255, 255, 255)

    #for draw_i in range(0, 128):
    #    cv2.circle(img, 10, 10, 10, (0, 0, 255), 1)
# cv2.imshow('image', img)
    #  k = cv2.waitKey(100) #& 0xFF
#   cv2.imwrite("circle.png", img)
    #   pdf.image("circle.png", x, y, circleDiameter, circleDiameter, 'png')
#    try:
#os.remove("/home/peter/FInCoR/src/circle_detection/src/circle_image/i" + str(2-9) + ".png")
    #   except: 
        #      pass
#pdf.output('BETA.pdf', 'F')






