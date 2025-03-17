import numpy as np
import rospy

def neighbour_count(map, x, y, width=None, height=None):
    if width is None:
        width = int((len(map[0])))
    if height is None:
        height = int((len(map)))

    count = 0
    walls = 0
    area_coefficient = 5
    for dx in [-area_coefficient, 0, area_coefficient]:
        for dy in [-area_coefficient, 0, area_coefficient]:
            nx, ny = x + dx, y + dy
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                idx = nx + ny * width
                if map[idx] == -1:
                    unknown_neighbors += 1
                elif map[idx] > 0:
                    walls += 1
    return max(count - walls*0.3, 0) / area_coefficient**2
    