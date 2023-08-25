import cv2
import numpy as np

image = cv2.imread('./Img/0002c.png')
overlay = cv2.imread('./Img/0002d.png')
# 设置半透明效果，alpha 值越小越透明
alpha = 0.5

overlay = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0, image)

font = cv2.FONT_HERSHEY_SIMPLEX
cv2.putText(overlay,'OpenCV',(100,100), font, 1,(0,255,255),1,cv2.LINE_AA)
#cv2.line(overlay, start_point, end_point, line_color, 2)

img = cv2.imwrite('./Img/0002e.png', overlay)
