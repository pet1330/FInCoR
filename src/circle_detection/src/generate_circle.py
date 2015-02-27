import cv2
from fpdf import FPDF
import numpy as np
import os
import sys

numOfCircles = 150
circleDiameter = 50; #in mm

xPos = 0;
yPos = 0;


pdf = FPDF()
pdf.alias_nb_pages()
pdf.add_page()

numberPerRow = (210 / (circleDiameter + (circleDiameter / 5)))
numberPerColum = (297 / (circleDiameter + (circleDiameter / 5)))
print '#######################################'
print numberPerRow, numberPerColum
print '#######################################'

img = np.ones((512, 512, 3), np.uint8)
for i in range(40, numOfCircles + 40):
    img[:, :] = (255, 255, 255)
    for draw_i in range(0, 128-i):
        cv2.circle(img, (256, 256), 252-(draw_i * 2), (0, 0, 0), 2)
    cv2.imwrite("circle.png", img)
    cv2.imshow('image', img)
    #k = cv2.waitKey(1000) & 0xFF
    
    pdf.image("circle.png", (xPos * (circleDiameter + (circleDiameter / 5)))-circleDiameter, (yPos * (circleDiameter + (circleDiameter / 5)))-circleDiameter, circleDiameter, circleDiameter, 'png')
    #print (xPos * (circleDiameter + (circleDiameter / 5)))-circleDiameter, (yPos * (circleDiameter + (circleDiameter / 5)))-circleDiameter

    xPos = (i % numberPerRow) + 1
    sys.stdout.write(str(xPos) + "  ")
    if (numberPerRow - xPos) == 0:
        yPos += 1
        print
        if yPos > numberPerColum:
            pdf.add_page()
            xPos = 0
            yPos = 0
            print "--------------"
    
pdf.output('BETA.pdf', 'F')