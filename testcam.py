import numpy as np
import sys
import cv2 as cv


def main():
    # Open Orbbec depth sensor
    orbbec_cap = cv.VideoCapture(0, cv.CAP_OBSENSOR)
    if orbbec_cap.isOpened() == False:
        sys.exit("Fail to open camera.")

    str = './Img/ObColor_0000.png&./Img/ObDepth_0000.png'  # 字符串序列
    color_png,  depth_png= str.split('&', 1)
    color_depth_png = './Img/ObColor_to_Depth_0000.png'

    while True:
        # Grab data from the camera
        if orbbec_cap.grab():
            # RGB data
            ret_bgr, bgr_image = orbbec_cap.retrieve(None, cv.CAP_OBSENSOR_BGR_IMAGE)
            if ret_bgr:
                #_, bgr_image = cv.imencode('.jpg', bgr_image)
                cv.imshow("BGR", bgr_image)
                cv.imwrite(color_png, bgr_image)

            # depth data
            ret_depth, depth_map = orbbec_cap.retrieve(None, cv.CAP_OBSENSOR_DEPTH_MAP)
            if ret_depth:
                cv.imwrite(depth_png, depth_map)
                color_depth_map = cv.normalize(depth_map, None, 0, 255, cv.NORM_MINMAX, cv.CV_8UC1)
                color_depth_map = cv.applyColorMap(color_depth_map, cv.COLORMAP_JET)
                cv.imshow("DEPTH", color_depth_map)
                cv.imwrite(color_depth_png, color_depth_map)
        else:
            print("Fail to grab data from the camera.")

        if cv.pollKey() >= 0:
            orbbec_cap.release()
            orbbec_cap = None
            break

    orbbec_cap.release()
    orbbec_cap = None


if __name__ == '__main__':
    main()