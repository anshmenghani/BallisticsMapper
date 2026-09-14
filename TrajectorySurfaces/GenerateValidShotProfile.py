'''
For each (r, vf, vl) grid point, finds every RPS setting that produces
a valid shot — i.e. phi and theta both fall within the mechanically
achievable angle bounds.

Output (CSV + NPZ):
  r, vf, vl, rps, phi_rad, theta_rad, phi_deg, theta_deg

One row per valid (r, vf, vl, rps) combination.
If multiple RPS options are valid for a given (r, vf, vl), all are kept
so downstream code can choose (e.g. prefer a specific RPS, pick mid-range phi).
'''

import numpy as np
import pandas as pd
import os

# Config Setups
BASE = 'ProjectileMotionSim/TrajectorySurfaces'
RPS_DIRS = {40: '40_RPS', 50: '50_RPS', 60: '60_RPS'}
OUT_DIR = 'ProjectileMotionSim/TrajectorySurfaces/valid_shot_table'

PHI_MIN, PHI_MAX = np.radians(45), np.radians(75)
THETA_MIN, THETA_MAX = np.radians(0), np.radians(340)

os.makedirs(OUT_DIR, exist_ok=True)

# Load all RPS datasets
datasets = {}
for rps, dirname in RPS_DIRS.items():
    path = f'{BASE}/{dirname}/clean_theta_phi_surface.npz'
    d = np.load(path)
    datasets[rps] = {
        'r_vals':  d['r_vals'],
        'vf_vals': d['vf_vals'],
        'vl_vals': d['vl_vals'],
        'theta': d['theta_surface'],
        'phi': d['phi_surface'],
    }
    print(f"Loaded {rps} RPS: grid {d['theta_surface'].shape}  "
          f"r=[{d['r_vals'].min():.2f}, {d['r_vals'].max():.2f}]  "
          f"vf=[{d['vf_vals'].min():.2f}, {d['vf_vals'].max():.2f}]  "
          f"vl=[{d['vl_vals'].min():.2f}, {d['vl_vals'].max():.2f}]")

# Verify all datasets share the same grid (required for a unified table)
ref = datasets[next(iter(datasets))]
for rps, d in datasets.items():
    for key in ('r_vals', 'vf_vals', 'vl_vals'):
        if not np.allclose(d[key], ref[key]):
            raise ValueError(
                f"RPS={rps} has different {key} grid — "
                "all datasets must share the same (r, vf, vl) axes."
            )
print("\nAll grids match. Building meshgrid...")

r_vals  = ref['r_vals']
vf_vals = ref['vf_vals']
vl_vals = ref['vl_vals']

R, VF, VL = np.meshgrid(r_vals, vf_vals, vl_vals, indexing='ij')
r_flat  = R.ravel()
vf_flat = VF.ravel()
vl_flat = VL.ravel()

# Collect valid rows
rows = []

for rps, d in datasets.items():
    phi_flat = d['phi'].ravel()
    theta_flat = d['theta'].ravel()

    valid = (
        np.isfinite(phi_flat) & np.isfinite(theta_flat) &
        (phi_flat >= PHI_MIN) & (phi_flat <= PHI_MAX) &
        (theta_flat >= THETA_MIN) & (theta_flat <= THETA_MAX)
    )

    n_valid = valid.sum()
    print(f"RPS={rps:2d}: {n_valid} valid points  "
          f"({valid.sum() / len(valid) * 100:.1f}% of grid)")

    for i in np.where(valid)[0]:
        rows.append({
            'r': float(r_flat[i]),
            'vf': float(vf_flat[i]),
            'vl': float(vl_flat[i]),
            'rps': int(rps),
            'phi_rad': float(phi_flat[i]),
            'theta_rad': float(theta_flat[i]),
            'phi_deg': float(np.degrees(phi_flat[i])),
            'theta_deg': float(np.degrees(theta_flat[i])),
        })

df = pd.DataFrame(rows).sort_values(['r', 'vf', 'vl', 'rps']).reset_index(drop=True)

# Coverage summary
total_triplets = len(r_flat)
unique_covered = df.groupby(['r', 'vf', 'vl']).ngroups
multi_rps = df.groupby(['r', 'vf', 'vl']).size()

print(f"\n{'='*55}")
print(f"Total (r,vf,vl) triplets in grid: {total_triplets}")
print(f"Triplets with at least one valid RPS: {unique_covered}")
print(f"Triplets with exactly 1 valid RPS: {(multi_rps == 1).sum()}")
print(f"Triplets with exactly 2 valid RPS: {(multi_rps == 2).sum()}")
print(f"Triplets with all 3 valid RPS: {(multi_rps == 3).sum()}")
print(f"Total rows in table: {len(df)}")
print(f"{'='*55}")

# Per-RPS row count
for rps in sorted(RPS_DIRS.keys()):
    n = (df['rps'] == rps).sum()
    print(f". RPS={rps}: {n} rows")

# Save outputs
csv_path = f'{OUT_DIR}/valid_shots.csv'
npz_path = f'{OUT_DIR}/valid_shots.npz'

df.to_csv(csv_path, index=False)

np.savez(npz_path,
    r = df['r'].to_numpy(),
    vf = df['vf'].to_numpy(),
    vl = df['vl'].to_numpy(),
    rps = df['rps'].to_numpy(),
    phi_rad  = df['phi_rad'].to_numpy(),
    theta_rad = df['theta_rad'].to_numpy(),
    phi_deg  = df['phi_deg'].to_numpy(),
    theta_deg = df['theta_deg'].to_numpy(),
    # metadata
    phi_bounds_deg = np.array([45,  75]),
    theta_bounds_deg = np.array([0,  340]),
    rps_values = np.array(sorted(RPS_DIRS.keys())),
)

print(f"\nSaved: {csv_path}")
print(f"Saved: {npz_path}")
print(f"\nFirst 10 rows:")
print(df.head(10).to_string(index=False))
