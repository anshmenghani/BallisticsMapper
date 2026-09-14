import numpy as np

# clearance for Hub in 2026 FRC game
def hub_clearance(vel_approx_tlist, vx_list, vy_list, vz_list, speedf, pos_approx_tlist, sx_list, sy_list, sz_list, omegaf): # solver will pass all these parameters, not all are used
    hub_height = 1.8288 # 6ft 6in, the target y, height of the Hub
    hub_half_diag = 0.5 # half diagonal distance from center of hub to corner
    fuel_diameter = 0.1524 
    extra_tolerance = 0.2 

    xc, zc = (10, 10) # hub center relative to origin
    required_height = hub_height + fuel_diameter + extra_tolerance # clearance height

    # if ball clears the edge of the hub plus the extra tolerance, return 0. otherwise, increase the error in the Levenberg–Marquardt algorithm
    for i in range(len(sx_list) - 1, -1, -1): # work backwards along trajectory from hub center
        dx = sx_list[i] - xc
        dz = sz_list[i] - zc
        xz_dist = np.hypot(dx, dz)

        if xz_dist >= hub_half_diag: # at hub edge 
            y = sy_list[i]
            delta = required_height - y
            return np.log(1 + np.exp(delta)) # if (required_height - y) is negative, the shot clears and there is no penalty. otherwise, a penalty of (required_height - y) is applied

    return required_height # big penalty if shot does not reach the hub
