import ctypes

# 加载共享库
lib1 = ctypes.CDLL('./build/libdemo1.so')  # 注意，路径可能需要根据实际情况进行修改

# 调用C++函数
lib1.CalCylinder()
 
