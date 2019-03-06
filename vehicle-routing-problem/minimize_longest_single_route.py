# https://developers.google.com/optimization/routing/vrp?hl=zh-TW
from matplotlib import pyplot as plt
import numpy as np

locations = np.array([
    [456, 320], # location 0
    [228, 0],    # location 1
    [912, 0],    # location 2
    [0, 80],     # location 3
    [114, 80],   # location 4
    [570, 160],  # location 5
    [798, 160],  # location 6
    [342, 240],  # location 7
    [684, 240],  # location 8
    [570, 400],  # location 9
    [912, 400],  # location 10
    [114, 480],  # location 11
    [228, 480],  # location 12
    [342, 560],  # location 13
    [684, 560],  # location 14
    [0, 640],    # location 15
    [798, 640]   # location 16
])

print(locations)
print(locations[:,0])

plt.scatter(locations[:,0], locations[:,1])
plt.show()