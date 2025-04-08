import numpy as np
import matplotlib.pyplot as plt

def read_pgm(file_path):
    global file_type

    with open(file_path, 'rb') as f:
        magic_number = f.readline().decode().strip()
        file_type = magic_number  # 更新全局的文件类型变量
        f.readline().decode().strip()
        width, height = map(int, f.readline().decode().strip().split())
        max_gray_value = int(f.readline().decode().strip())

        if magic_number == 'P5':
            # 二进制格式
            pixels = list(f.read())
        elif magic_number == 'P2':
            # ASCII格式
            pixels = [int(line) for line in f.readlines() if line.strip()]
        else:
            raise ValueError("Unsupported file type.")
        
        pixels_array = np.array(pixels).reshape(height, width)

        out = pixels_array >=250  # 输出 0/1 黑白图

        return (out)

if __name__ == '__main__':
    file_path = r'./map/my_vehicle.pgm'

    pixels_array = read_pgm(file_path)

