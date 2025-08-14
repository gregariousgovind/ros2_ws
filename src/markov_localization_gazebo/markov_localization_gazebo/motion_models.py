import numpy as np

def odom_to_delta(prev, curr):
    # prev/curr: (x,y,theta)
    dx = curr[0] - prev[0]
    dy = curr[1] - prev[1]
    dtrans = np.hypot(dx, dy)
    dtheta = np.arctan2(np.sin(curr[2]-prev[2]), np.cos(curr[2]-prev[2]))
    drot1 = np.arctan2(dy, dx) - prev[2]
    drot2 = dtheta - drot1
    return dtrans, drot1, drot2

def motion_kernel(theta_bins, trans_sigma, rot_sigma, trans_rot_sigma, res):
    # Build separable kernels over (dx, dy, dtheta) for small motions
    # We approximate by Gaussian over continuous motion projected to discrete bins.
    # Return small convolution stencils.
    max_cells = max(1, int(3*trans_sigma/res))
    xs = np.arange(-max_cells, max_cells+1)
    Kxy = np.exp(-0.5*(xs*res)**2/(trans_sigma**2 + 1e-6))
    Kxy /= Kxy.sum()
    thetas = np.arange(-2, 3)  # +-20 deg range if 10 deg bins
    Kth = np.exp(-0.5*(np.deg2rad(thetas*10.0))**2/(rot_sigma**2 + 1e-6))
    Kth /= Kth.sum()
    return xs, Kxy, thetas, Kth
