import cv2
from fpdf import FPDF
import math
import numpy as np
import sys

circleDiameter = 50

numOfCircles = 10
numberPerRow = (210 / (circleDiameter + (circleDiameter / 5)))
numberPerColum = (297 / (circleDiameter + (circleDiameter / 5)))



def gen_single_circle(outsideCircleRad, insideCircleRad):
    
    scale = outsideCircleRad;
    
    img = np.ones(((scale * 2) + 10, (scale * 2) + 10, 3), np.uint8)
    img[:,:] = (255, 255, 255)
    
    for i in range(outsideCircleRad, insideCircleRad, -2):
        cv2.circle(img, (scale + 5, scale + 5), i, (0, 0, 0), 2)
    return img


def gen_pdf():
    
    pdf = FPDF()
    pdf.add_page()

    for i in range(1, numOfCircles):
        xPos = 0
        yPos = 0        
        cv2.imwrite('circle.png', gen_single_circle(circleDiameter, 30))
        
        xPos += 1
        if xPos > numberPerRow:
            yPos += 1
            xPos = 0
        if yPos > numberPerColum:
            pdf.add_page()
            xPos = 0
            yPos = 0
        
        pdf.image('circle.png', (xPos * (circleDiameter + (circleDiameter / 5)))-circleDiameter, (yPos * (circleDiameter + (circleDiameter / 5)))-circleDiameter, circleDiameter, circleDiameter, 'png')

        #cv2.imshow('image', a);
        #k = cv2.waitKey(1000) & 0xFF

    pdf.output('BETA.pdf', 'F')
        
gen_pdf()
#cv2.imshow('image', a);
#k = cv2.waitKey(1000) & 0xFF