import numpy as np
from .grid_utils import world_to_grid, bresenham

def integrate_scan_logodds(logodds, origin_x, origin_y, res, x, y, th, angles, ranges, max_range, hit_logit, miss_logit, free_decay):
    gx0, gy0 = world_to_grid(x, y, origin_x, origin_y, res)
    H, W = logodds.shape
    for a, r in zip(angles, ranges):
        r_eff = min(r, max_range)
        gx1 = gx0 + int((r_eff/res)*np.cos(th+a))
        gy1 = gy0 + int((r_eff/res)*np.sin(th+a))
        cells = bresenham(gx0, gy0, gx1, gy1)
        for (cx, cy) in cells[:-1]:
            if 0 <= cx < W and 0 <= cy < H:
                logodds[cy, cx] = free_decay*logodds[cy, cx] + miss_logit
        # end cell
        cx, cy = cells[-1]
        if 0 <= cx < W and 0 <= cy < H and r < max_range:
            logodds[cy, cx] = free_decay*logodds[cy, cx] + hit_logit
    return logodds

def logodds_to_occ(logodds):
    p = 1.0 - 1.0/(1.0 + np.exp(logodds))
    occ = (p*100).astype('int16')
    occ = occ.clip(0, 100)
    return occ
