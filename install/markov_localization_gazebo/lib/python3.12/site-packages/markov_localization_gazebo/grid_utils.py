import numpy as np

def world_to_grid(x, y, origin_x, origin_y, res):
    gx = int((x - origin_x) / res)
    gy = int((y - origin_y) / res)
    return gx, gy

def grid_to_world(gx, gy, origin_x, origin_y, res):
    x = gx * res + origin_x + 0.5 * res
    y = gy * res + origin_y + 0.5 * res
    return x, y

def bresenham(x0, y0, x1, y1):
    # integer Bresenham
    dx = abs(x1 - x0); sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    cells = []
    while True:
        cells.append((x, y))
        if x == x1 and y == y1: break
        e2 = 2 * err
        if e2 >= dy: err += dy; x += sx
        if e2 <= dx: err += dx; y += sy
    return cells
