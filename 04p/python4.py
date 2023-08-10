import ctypes

# 加载共享库
lib4 = ctypes.CDLL('./build/libdemo4.so')  # 注意，路径可能需要根据实际情况进行修改

# 调用C++函数
lib4.USBReboot()

