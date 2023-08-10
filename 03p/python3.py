import ctypes

# 加载共享库
lib3 = ctypes.CDLL('./build/libdemo3.so')  # 注意，路径可能需要根据实际情况进行修改

# 调用C++函数
frameset = lib3.framePipeline()
lib3.saveDepthImg(frameset)

