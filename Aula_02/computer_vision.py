# -*- coding:utf-8 -*-

import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

caixa_h = 36.3
# Para usar o vídeo
#cap = cv2.VideoCapture('hall_box_battery_1024.mp4')

# As 3 próximas linhas são para usar a webcam
cap = cv2.VideoCapture(0)
time.sleep(1.2)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480)

def identifica_cor(frame):
    '''
    Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
    '''

    # No OpenCV, o canal H vai de 0 até 179, logo cores similares ao 
    # vermelho puro (H=0) estão entre H=-8 e H=8. 
    # Veja se este intervalo de cores está bom
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #cor_menor = np.array([0, 50, 50])
    #cor_maior = np.array([8, 255, 255])
    #segmentado_cor1 = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    
    cor_menor = np.array([170, 50, 50])
    cor_maior = np.array([179, 255, 255])
    segmentado_cor2 = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    segmentado_cor = segmentado_cor2
    
    # Será possível limpar a imagem segmentado_cor? 
    # Pesquise: https://docs.opencv.org/trunk/d9/d61/tutorial_py_morphological_ops.html

    kernel = np.ones((5,5),np.uint8)
    segmentado_cor = cv2.erode(segmentado_cor,kernel,iterations = 3)
    segmentado_cor = cv2.dilate(segmentado_cor,kernel,iterations = 1)
    
    
    # Encontramos os contornos na máscara e selecionamos o de maior área
    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    maior_contorno = None
    maior_contorno_area = 0

    cv2.drawContours(frame, contornos, -1, [255, 0, 255], 5)


    for cnt in contornos:
        area = cv2.contourArea(cnt)


        if area > maior_contorno_area:
            maior_contorno = cnt
            maior_contorno_area = area

    
    # Encontramos o centro do contorno fazendo a média de todos seus pontos.
    if not maior_contorno is None :
        cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(frame, tuple(media), 5, [0, 255, 0])
        
        
        maxY = max(maior_contorno[:,1])
        minY = min(maior_contorno[:,1])
        px = maxY-minY
        dx = 660
        d = dx*caixa_h/px
        print("A distancia eh: ",d)

            
    else:
        media = (0, 0)
        d = 0

    cv2.imshow('', frame)
    cv2.imshow('imagem in_range', segmentado_cor)
    cv2.waitKey(1)

    centro = (frame.shape[0]//2, frame.shape[1]//2)

    return media, centro, d


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()


    img = frame.copy()

    media, centro,d = identifica_cor(img)

    #More drawing functions @ http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html

    # Display the resulting frame
    font = cv2.FONT_HERSHEY_SIMPLEX
    titulo = str(int(d)) + "cm"
    cv2.putText(frame,titulo,(0,475),font,2,(255,0,255),3,cv2.CV_AA)
    cv2.imshow('original',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

