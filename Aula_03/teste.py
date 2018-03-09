#!/usr/bin/env python
# -*- coding:utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

# If you want to open a video, just change this path
#cap = cv2.VideoCapture('hall_box_battery.mp4')

# Parameters to use when opening the webcam.
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


def auto_canny(image, sigma=0.38):

    lower = 0
    upper = 1

    # Returns an image containing the borders of the image
    # sigma is how far from the median we are setting the thresholds

    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

MIN_MATCH_COUNT = 60

img1 = cv2.imread('madfox.png',0)# Imagem a procurar

sift = cv2.xfeatures2d.SIFT_create()


while(True):
    ret, img2 = cap.read()
    # Save an colored image to use as output
    img2_color = img2.copy()
    # Convert the frame to grayscale
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    # A gaussian blur to get rid of the noise in the image
    blur = cv2.GaussianBlur(img2,(5,5),0)
    # Detect the edges present in the image
    bordas = auto_canny(blur)
    circles = []

    # HoughCircles - detects circles using the Hough Method. For an explanation of
    # param1 and param2 please see an explanation here http://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/
    circles = None
    circles=cv2.HoughCircles(bordas,cv2.HOUGH_GRADIENT,2,40,param1=50,param2=150,minRadius=5,maxRadius=60)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
            cv2.circle(img2_color,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(img2_color,(i[0],i[1]),2,(0,0,255),3)
            #cv2.line(img2,(0,0),(i[0],i[1]),(255,0,0),5)


        #Depois de detectar os círculos, passamos as imagens originais para serem comparadas pelo sift
        kp1, des1 = sift.detectAndCompute(img1,None)
        kp2, des2 = sift.detectAndCompute(img2,None)
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        # Configura o algoritmo de casamento de features
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        # Tenta fazer a melhor comparacao usando o algoritmo
        matches = flann.knnMatch(des1,des2,k=2)
        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)
        if(len(circles[0]) > 1):
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img2_color,'More than one circle detected',(20,460), font, 1,(0,255,255),2,cv2.LINE_AA)

        elif len(good)>MIN_MATCH_COUNT:

            #print('ACHEEEEEEEEEEEEEEEEEEEEEEEEEEEEEI a foooolhaaa')
            font = cv2.FONT_HERSHEY_SIMPLEX
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)


            # Tenta achar uma trasformacao composta de rotacao, translacao e escala que situe uma imagem na outra
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w = img1.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

            # Transforma os pontos da imagem origem para onde estao na imagem destino
            dst = cv2.perspectiveTransform(pts,M)
            # Desenha as linhas
            cv2.polylines(img2_color,[np.int32(dst)],True,(0,0,255),3, cv2.LINE_AA)

            #desenha o centro do polígono
            #pol_y = np.int32((dst[1][0][1] - dst[0][0][1])/2 + dst[0][0][1])
            #pol_x = np.int32((dst[3][0][1] - dst[0][0][0])/2 + dst[0][0][1])
            pol_x = np.int32(dst[0][0][0])
            circ_x = circles[0][0][0]
            if(pol_x-circ_x < 0):
                cv2.putText(img2_color,'Circulo atras da raposa',(20,460), font, 1,(0,255,0),2,cv2.LINE_AA)
            else:
                cv2.putText(img2_color,'Circulo na frente da raposinha',(20,460), font, 1,(0,255,0),2,cv2.LINE_AA)

        else:
            #print("Achei o circulo, mas nao a raposa")
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img2_color,'So falta a raposinha',(20,460), font, 1,(0,255,255),2,cv2.LINE_AA)


    else:
        #print("Não achei o circulo.. Preciso dele para funcioar :'( ")
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img2_color,'Gibbe circulos',(20,460), font, 1,(20,0,255),2,cv2.LINE_AA)
    # Display the resulting frame
    cv2.imshow('Detector de circulos',img2_color)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
