import numpy as np
import csv
from noise import pnoise2

# 设置高度场的分辨率和尺寸
size_x = 20  # 20m
size_y = 1   # 1m
resolution = 20

# 设置 Perlin 噪声的参数
octaves = 6
persistence = 0.5
lacunarity = 2.0
point_x = size_x*resolution
point_y = size_y*resolution

# 生成 Perlin 噪声高度数据
height_data = np.zeros((point_x, point_y))
for i in range(point_x):
    for j in range(point_y):
        height_data[i][j] = pnoise2(i*0.1,
                                    j*0.1,
                                    octaves=octaves,
                                    persistence=persistence,
                                    lacunarity=lacunarity,
                                    base=4)

# 将噪声值从 -1 到 1 的范围映射到 0 到 1 的范围
height_data = (height_data + 1) / 2

# 将噪声值缩放到 0 到 0.1 的范围
height_data = height_data * 0.1

# 将高度数据保存为CSV文件
with open('../../lidar_data_handler1/perlin_heightfield.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    for row in height_data:
        writer.writerow(row)

print('Perlin noise heightfield data saved to perlin_heightfield.csv')