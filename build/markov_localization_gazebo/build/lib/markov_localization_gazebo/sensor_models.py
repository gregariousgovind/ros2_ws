import numpy as np
from .grid_utils import world_to_grid, bresenham

def expected_ranges_from_map(occ_grid, origin_x, origin_y, res, x, y, th, angles, max_range):
    grid = occ_grid  # 0..100, -1 unknown
    gx0, gy0 = world_to_grid(x, y, origin_x, origin_y, res)
    ranges = []
    for a in angles:
        gx1 = gx0 + int((max_range/res)*np.cos(th+a))
        gy1 = gy0 + int((max_range/res)*np.sin(th+a))
        cells = bresenham(gx0, gy0, gx1, gy1)
        r = max_range
        for (cx, cy) in cells:
            if cx < 0 or cy < 0 or cx >= grid.shape[1] or cy >= grid.shape[0]:
                break
            if grid[cy, cx] > 50: # occupied
                dx = (cx-gx0)*res; dy = (cy-gy0)*res
                r = np.hypot(dx, dy)
                break
        ranges.append(r)
    return np.array(ranges)

def likelihood_beam_model(z_exp, z_obs, z_hit=0.9, z_rand=0.1, sigma_hit=0.2, max_range=3.5):
    p_hit = (1/np.sqrt(2*np.pi*sigma_hit**2)) * np.exp(-0.5*((z_obs - z_exp)**2)/(sigma_hit**2))
    p_rand = 1.0/max_range * np.ones_like(z_obs)
    return z_hit*p_hit + z_rand*p_rand
