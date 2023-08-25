import ctypes

# 加载共享库
lib5 = ctypes.CDLL('./build/libdemo5.so')  # 注意，路径可能需要根据实际情况进行修改

a1 = 443453
b1 = 54452

# 调用C++函数
c1 = lib5.IntCal(a1, b1)

print(c1)


 