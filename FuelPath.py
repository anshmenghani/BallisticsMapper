'''
Picks a random sample from valid_trajectories.csv and runs each
through the ProjectileSolver to verify the shot actually lands.

CSV field mapping:
  r   -> xt (distance to hub along x, target at xt, yt=1.8288, zt=0)
  vf  -> vx_robot (forward velocity toward target)
  vl  -> vz_robot (lateral velocity)
  rps -> omegai0 (converted to rad/s via * 2π)
'''

import csv
import random
import time
import math
import numpy as np
import ProjectilePath as pp
import FuelClearance as fc
from numpy import array, rad2deg, linalg, pi, sin
from tabulate import tabulate

# Config
N_SAMPLES = 20
RANDOM_SEED = 42

Y_TARGET = 1.8288 # hub height (m)
Y0 = 0.47 # launch height (m)
VY_ROBOT = 0 # robot has no vertical velocity

def get_frc900_spin_and_speed_from_shooter_rps(rps):
    speed_in_rps = np.abs((1 - 0.233333333) * rps)
    speed = speed_in_rps * 0.31  # m/s

    spin_in_rps = np.abs(0.233333333 * rps)
    spin = spin_in_rps * 2 * np.pi  # rad/s, backspin is positive

    return speed, spin

# Load and sample
with open('ProjectileMotionSim/TrajectorySurfaces/valid_trajectories_mirrored.csv') as f:
    rows = list(csv.DictReader(f))

random.seed(RANDOM_SEED)
sample = random.sample(rows, N_SAMPLES)

fuel = pp.Projectile(0.0762, 0.226796)
good = 0
misses = []

for i, row in enumerate(sample):
    r = float(row['r'])
    vf = float(row['vf'])
    vl = float(row['vl'])
    rps = float(row['rps'])
    phi_exp  = float(row['phi_rad'])
    theta_exp = float(row['theta_rad'])

    xt = r
    vx_robot = vf
    vz_robot = vl

    speed, omega = get_frc900_spin_and_speed_from_shooter_rps(rps)
    vy_seed = speed * sin(phi_exp)

    v_mag_robot = linalg.norm(array([vx_robot, VY_ROBOT, vz_robot]))

    print(f"\n{'='*60}")
    print(f"Sample {i+1}/{N_SAMPLES}  |  r={r}m  vf={vf}  vl={vl}  rps={rps}")
    print(f"Expected  phi={rad2deg(phi_exp):.2f}°  theta={rad2deg(theta_exp):.2f}°  vy_seed={vy_seed:.3f}")

    fuel_solver = pp.ProjectileSolver(
        fuel, xt, Y_TARGET, 0,
        vx_robot, vy_seed, vz_robot, omega,
        sy0=Y0,
        vx_frame=vx_robot, vy_frame=VY_ROBOT, vz_frame=vz_robot,
        fix_speed=True, fix_omega=True,
        lm_iters=20, sim_end_time=5, dt=0.01,
        clearance_func=fc.hub_clearance,
        phi_bounds=(0.785, 1.309),
        theta_bounds=(-3.05, 3.05)
    )

    start = time.perf_counter()
    fuel_solver.levenberg_marquardt()
    elapsed = time.perf_counter() - start

    _, vx_list, vy_list, vz_list, _, pos_approx_tlist, sx_list, sy_list, sz_list, _ = fuel.trajectory(
        fuel_solver.vx + vx_robot,
        fuel_solver.vy + VY_ROBOT,
        fuel_solver.vz + vz_robot,
        fuel_solver.omega,
        sy0=Y0, dt=0.01, stop_on_y=Y_TARGET
    )

    dx = sx_list[-1] - xt
    dz = sz_list[-1]
    dist_2d = math.hypot(dx, dz)
    landed = dist_2d < 0.3

    phi_err = rad2deg(fuel_solver.phi) - rad2deg(phi_exp)
    theta_err = rad2deg(fuel_solver.theta) - rad2deg(theta_exp)

    table = [
        ["LM Time (ms)", 1000 * elapsed],
        ["Expected phi (deg)", rad2deg(phi_exp)],
        ["Solved phi (deg)", rad2deg(fuel_solver.phi)],
        ["Phi error (deg)", f"{phi_err:+.3f}"],
        ["Expected theta (deg)", rad2deg(theta_exp)],
        ["Solved theta (deg)", rad2deg(fuel_solver.theta)],
        ["Theta error (deg)", f"{theta_err:+.3f}"],
        ["Shot Speed (m/s)", speed],
        ["Shot Spin (rad/s)", omega],
        ["Total vx (m/s)", fuel_solver.vx + vx_robot],
        ["Total vy (m/s)", fuel_solver.vy + VY_ROBOT],
        ["Total vz (m/s)", fuel_solver.vz + vz_robot],
        ["Final X (m)", sx_list[-1]],
        ["Final Y (m)", sy_list[-1]],
        ["Final Z (m)", sz_list[-1]],
        ["Target X (m)", xt],
        ["Error X (m)", f"{dx:+.4f}"],
        ["Error Z (m)", f"{dz:+.4f}"],
        ["Distance from target (m)", f"{dist_2d:.4f}"],
        ["Landed in target?", landed],
    ]

    if landed:
        good += 1
    else:
        misses.append({
            '#': i + 1,
            'r': r,  'vf': vf,  'vl': vl,  'rps': rps,
            'err_x': dx, 'err_z': dz, 'dist': dist_2d,
            'phi_err': phi_err, 'theta_err': theta_err,
        })

    print(tabulate(table, headers=["Parameter", "Value"], tablefmt="rounded_grid"))

# Summary
print(f"\nAccuracy: {good}/{N_SAMPLES} -- {good/N_SAMPLES:.0%}")

if misses:
    print(f"\nMiss summary ({len(misses)} misses):")
    miss_rows = [
        [m['#'], m['r'], m['vf'], m['vl'], m['rps'],
         f"{m['err_x']:+.3f}", f"{m['err_z']:+.3f}", f"{m['dist']:.3f}",
         f"{m['phi_err']:+.2f}°", f"{m['theta_err']:+.2f}°"]
        for m in misses
    ]
    print(tabulate(miss_rows,
                   headers=["#", "r", "vf", "vl", "rps",
                             "err_x(m)", "err_z(m)", "dist(m)",
                             "phi_err", "theta_err"],
                   tablefmt="rounded_grid"))
    