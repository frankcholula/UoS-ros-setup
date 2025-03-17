import numpy as np

def neighbour_count(map, x, y, width=None, height=None):
    if width is None:
        width = int((len(map[0])))
    if height is None:
        height = int((len(map)))

    count = 0
    area_coefficient = 8
    for dx in [-area_coefficient, 0, area_coefficient]:
        for dy in [-area_coefficient, 0, area_coefficient]:
            nx, ny = x + dx, y + dy
            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                idx = nx + ny * width
                if map[idx] == -1:
                    unknown_neighbors += 1
    return count / area_coefficient**2
    