import numpy as np
def clamp(v, a, b): return max(a, min(b, v))
def world_to_grid(x, y, ox, oy, res):
    return int((x-ox)/res), int((y-oy)/res)
def grid_to_world(gx, gy, ox, oy, res):
    return gx*res + ox + 0.5*res, gy*res + oy + 0.5*res
def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0); sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
    err = dx + dy; x, y = x0, y0
    pts=[]
    while True:
        pts.append((x,y))
        if x==x1 and y==y1: break
        e2=2*err
        if e2>=dy: err+=dy; x+=sx
        if e2<=dx: err+=dx; y+=sy
    return pts
