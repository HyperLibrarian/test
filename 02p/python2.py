import ctypes

# 加载共享库
lib2 = ctypes.CDLL('./build/libdemo2.so')  # 注意，路径可能需要根据实际情况进行修改

# 调用C++函数
result = lib2.SaveImg()
result = result.decode('utf-8')


original_string = "part1&part2"

# 使用split()方法拆分字符串
parts = original_string.split("&", 1)

# 分别赋值给两个变量
part1, part2 = parts[0], parts[1]
print("part1:", part1)  # Output: part1: part1
print("part2:", part2)  # Output: part2: part2
print(result) 
