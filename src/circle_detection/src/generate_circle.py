import cv2
from fpdf import FPDF
import math
import numpy as np
import sys

circleDiameter = 21
numOfCircles = 100
numberPerRow = (210 / (circleDiameter + (circleDiameter / 5)))
numberPerColum = (297 / (circleDiameter + (circleDiameter / 5)))

def gen_single_circle(outsideCircleRad, insideCircleRad):
    # EDIT THIS TO BE CLASS VAR circleDiameter INSTEAD OF 50
    increments = (outsideCircleRad - insideCircleRad) / 50
    scale = outsideCircleRad
    img = np.ones(((scale * 2) + 10, (scale * 2) + 10, 3), np.uint8)
    img[:,:] = (255, 255, 255)
    
    for i in range(outsideCircleRad, insideCircleRad, -increments):
        cv2.circle(img, (scale + 5, scale + 5), i, (0, 0, 0), increments)
    return img

def gen_pdf():
    pdf = FPDF()
    pdf.add_page()
    xPos = 0
    yPos = 1
    print numberPerRow, numberPerColum
    print "############################"
    for i in range(1, numOfCircles+1):
        cv2.imwrite('circle.png', gen_single_circle(500, 200))
        k = cv2.waitKey(500) & 0xFF
        xPos += 1
        x = (xPos * (circleDiameter + (circleDiameter / 5)))-circleDiameter
        y = (yPos * (circleDiameter + (circleDiameter / 5)))
#x = (xPos * (circleDiameter + (circleDiameter / 5)))-circleDiameter
#y = (yPos * (circleDiameter + (circleDiameter / 5)))-circleDiameter
        sys.stdout.write('(' + str(x) + ' , ' + str(y) + ')    ')
        pdf.image('circle.png',x,y, circleDiameter, circleDiameter, 'png')    
        
        if xPos > numberPerRow-1:
            print ""
            yPos += 1
            xPos = 0
        if yPos > numberPerColum:
            if (i != (numOfCircles)):
                pdf.add_page()
                print "-------------------------------------"
                xPos = 0
                yPos = 1
    pdf.output('BETA.pdf', 'F')
        
gen_pdf()
#cv2.imshow('image', a);
#k = cv2.waitKey(1000) & 0xFF