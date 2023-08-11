from flask import Flask, render_template, Response,request, render_template, json, jsonify, send_file
import ctypes
import cv2
import io
import traceback
import os
import multiprocessing as mp
import time

camera_g = None
status = 1

class VideoCamera():
    def __init__(self):
        # 通过opencv获取实时视频流
        self.video = cv2.VideoCapture(0) 
        print(os.getpid())
        print(0, self.video.isOpened()) 
        global status
        if self.video.isOpened() == False :
            status = 0
            print(00)
    
    def __del__(self):
        print(1)
        self.video.release()
        #time.sleep(3)
          

    def get_frame(self):
        # print(2)
        # print(self.video.isOpened())
        success, image = self.video.read()
        # print(3)
        # 因为opencv读取的图片并非jpeg格式，因此要用motion JPEG模式需要先将图片转码成jpg格式图片
        if image is not None:
            # print(8)
            ret, jpeg = cv2.imencode('.jpg', image)
            return jpeg.tobytes()
        else:
            # print(9)
            return None
        
    def release(self):
        self.video.release()

def gen(camera):

    result = True
    while result:
        # print(5)
        frame = camera.get_frame()
        # cv2.waitKey(24)
        # 使用generator函数输出视频流， 每次请求输出的content类型是image/jpeg
        if frame:
            # print(6)
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
        else:
            # print(7)
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + b'\r\n\r\n')


app = Flask(__name__)

@app.route('/api/video/video_feed', methods=['GET'])  # 主页
def index():
    # jinja2模板，具体格式保存在index.html文件中
    return render_template('index.html')

@app.route('/api/video/video_feed/source', methods=['GET'])  # 返回视频流响应
def video_feed():
    print(123)
    global status
    global camera_g
    camera_g = None

    #if status == 0:
    if 1 == 0:
        response = {
        "code":0,
        "msg":"相机刚才进行拍摄或暂停",
        "data": status
        }
        status = 1
        return jsonify(response)


    else:
        camera = VideoCamera()
        camera_g = camera
        print(222, camera_g)
        return Response(gen(camera),
                        mimetype='multipart/x-mixed-replace; boundary=frame') 


  

@app.route('/api/video/stop_stream', methods=['GET'])   # 设备关闭
def stop_stream():
    
    #VideoCamera().stop_stream
    #lib0 = ctypes.CDLL('./lib/libdemo4.so')  # 加载共享库
    #lib0.USBReboot()
    #video0 = cv2.VideoCapture(1) 
    result = True
    global camera_g
    global status
    print(111, camera_g)
    if camera_g:
        camera_g.release()
        camera_g = None
        status = 0
        result = False
    
    response = {
        "code":0,
        "msg":"相机暂停",
        "data": result
    }
    return jsonify(response)

@app.route('/api/img/viewer', methods=['POST'])
def viewer():   # 显示图片
    
    data = request.get_data()
    data = json.loads(data)
    image_path = data['data'][0]['img_png']# 本地图片路径
    #image_path = './Result/Seg_0000.png'
    # 二进制形式读取图片文件
    with open(image_path, 'rb') as f:
        image_binary = f.read()
    # 返回图片流，设置mimetype为'image/png'
    return send_file(
        io.BytesIO(image_binary),
        mimetype='image/png'
    )


@app.route('/api/save/capture', methods=['GET'])
def capture():   # 拍摄
    
    lib2 = ctypes.CDLL('./lib/libdemo2.so')  # 加载共享库
    # lib4 = ctypes.CDLL('./lib/libdemo4.so')  # 加载共享库
    # lib4.USBReboot()
    # lib2.SaveImg()
    # lib2.SaveImg()
    # lib4.USBReboot()

    str = './Img/Color_0000.png&./Img/Depth_0000.png'  # 字符串序列
    color_png,  depth_png= str.split('&', 1)

    global camera_g
    global status
    if camera_g:
        camera_g.release()
        camera_g = None
        status = 0
    video1 = cv2.VideoCapture(0) 
    _, frame1 = video1.read()
    cv2.imwrite(color_png, frame1)
    video1.release()
    video1 = None

    #video2 = cv2.VideoCapture(1) 
    #_, frame2 = video2.read()
    #cv2.imwrite(depth_png, frame2)
    #video2.release()

    response = {
        "code":0,
        "msg":"拍摄成功",
        "data": [{"color_png":color_png,"depth_png": depth_png}]
    }
    return jsonify(response)

@app.route('/api/save/segment', methods=['POST'])
def segment():   # 测距
    
    data = request.get_data()
    data = json.loads(data)
    depth_path = data['data'][0]['depth_png']# 本地图片路径
    print(depth_path)
    #lib1 = ctypes.CDLL('./lib/libdemo1.so')  # 加载共享库
    #lib1.CalCylinder()

    seg_png = './Result/Seg_0000.png'# 本地图片路径
        # 将布尔值封装为JSON格式并返回
    response = {
        "code":0,
        "msg":"计算成功",
        "data": [{"color_png":seg_png}]
    }
    return jsonify(response)


# @app.route('/getsql')
#def get_sql()



if __name__ == '__main__':
    app.run(host = '0.0.0.0', port = 5001, debug = False)  