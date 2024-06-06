from Queue import Empty

import numpy as np
import multiprocessing


def hex2rgb(hex_value):
    h = hex_value.strip("#")
    rgb = tuple(int(h[i:i+2], 16) for i in (0, 2, 4))
    return rgb


def rgb2hsv(r, g, b):
    # Normalize R, G, B values
    r, g, b = r / 255.0, g / 255.0, b / 255.0

    # h, s, v = hue, saturation, value
    max_rgb = max(r, g, b)
    min_rgb = min(r, g, b)
    difference = max_rgb - min_rgb

    # if max_rgb and max_rgb are equal then h = 0
    if max_rgb == min_rgb:
        h = 0

    # if max_rgb==r then h is computed as follows
    elif max_rgb == r:
        h = (60 * ((g - b) / difference) + 360) % 360

    # if max_rgb==g then compute h as follows
    elif max_rgb == g:
        h = (60 * ((b - r) / difference) + 120) % 360

    # if max_rgb=b then compute h
    elif max_rgb == b:
        h = (60 * ((r - g) / difference) + 240) % 360

    # if max_rgb==zero then s=0
    if max_rgb == 0:
        s = 0
    else:
        s = (difference / max_rgb) * 100

    # compute v
    v = max_rgb * 100
    # return rounded values of H, S and V
    return tuple(map(round, (h, s, v)))

def point(x, y):
    # type: (float, float) -> np.ndarray
    return np.array([x, y])


def point_int(x, y):
    # type: (float, float) -> np.ndarray
    return np.array([x, y]).astype(int)


import math


def get_remaining(current_pos):
    positions = [(1, 2), (2, 3), (4, 3), (2, 1)]

    max_dist = 1  # in case no position is closer than max_dist, just return the entire positions list

    best_index = 0
    best_dist = max_dist

    for i in range(len(positions)):
        x1, y1 = current_pos
        x2, y2 = positions[i]
        dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

        if dist < best_dist:
            best_dist = dist
            best_index = i
    return positions[best_index:]



def clear_queu_and_send(queue, data):

    # type: (multiprocessing.Queue, _) -> None
    # while not queue.empty() > 0:
    #     queue.get()
    while not queue.empty():
        try:
            queue.get_nowait()
        except Empty:
            pass
    queue.put(data)