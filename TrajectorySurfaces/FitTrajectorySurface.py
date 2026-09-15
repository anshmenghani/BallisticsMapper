'''
Uses the simulated surfaces to generate a polynomial to fit them.
Drops NaNs, removes outliers, clamps to physically valid angle ranges,
then fits polynomials.

Valid angle ranges (set by physical / mechanical constraints):
  phi   : 45 deg  – 75 deg   (0.7854 – 1.3090 rad)  launch elevation
  theta : 0  deg  – 340 deg  (0.0000 – 5.9341 rad)   azimuth

phi is fit on (r, vf, vl) — all three matter.
theta is fit on (r, vl)   — essentially independent of vf.
'''

import numpy as np
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import Pipeline
from sklearn.metrics import r2_score
import pickle
import json
import os

# Valid angle bounds (radians)
PHI_MIN   = np.radians(45)    # 0.7854 rad
PHI_MAX   = np.radians(75)    # 1.3090 rad
THETA_MIN = np.radians(0)     # 0.0000 rad
THETA_MAX = np.radians(340)   # 5.9341 rad


def compute_valid_bounds(r, vf, vl):
    '''
    Returns global bounding box + per-r-slice boxes for the clean,
    in-range points. Save alongside the model for inference-time range checks.
    '''
    per_r = {}
    for rv in np.unique(r):
        mask = r == rv
        if mask.sum() < 3:
            continue
        per_r[f'{rv:.4f}'] = {
            'vf': [float(vf[mask].min()), float(vf[mask].max())],
            'vl': [float(vl[mask].min()), float(vl[mask].max())],
            'n_points': int(mask.sum()),
        }
    return {
        'global': {
            'r':  [float(r.min()), float(r.max())],
            'vf': [float(vf.min()), float(vf.max())],
            'vl': [float(vl.min()), float(vl.max())],
        },
        'angle_bounds': {
            'phi_deg':   [45, 75],
            'theta_deg': [0, 340],
            'phi_rad':   [PHI_MIN, PHI_MAX],
            'theta_rad': [THETA_MIN, THETA_MAX],
        },
        'per_r': per_r,
    }


def fit_surfaces(npz_path, name, poly_degree=4):
    data = np.load(npz_path)
    r_vals, vf_vals, vl_vals = data['r_vals'], data['vf_vals'], data['vl_vals']
    theta, phi = data['theta_surface'], data['phi_surface']

    # Drop NaNs
    R, VF, VL = np.meshgrid(r_vals, vf_vals, vl_vals, indexing='ij')
    valid = np.isfinite(theta) & np.isfinite(phi)
    r, vf, vl = R[valid], VF[valid], VL[valid]
    theta_all, phi_all = theta[valid], phi[valid]
    print(f'{name} -- After NaN drop: {len(r)} points')

    # Clamp to valid angle ranges
    in_range = (
        (phi_all >= PHI_MIN) & (phi_all <= PHI_MAX) &
        (theta_all >= THETA_MIN) & (theta_all <= THETA_MAX)
    )
    n_clamped = (~in_range).sum()
    r, vf, vl = r[in_range], vf[in_range], vl[in_range]
    theta_all, phi_all = theta_all[in_range],  phi_all[in_range]
    print(f'{name} -- After angle clamp: {len(r)} points ({n_clamped} dropped  '
          f'| phi [{np.degrees(PHI_MIN):.0f}°–{np.degrees(PHI_MAX):.0f}°]  '
          f'theta [{np.degrees(THETA_MIN):.0f}°–{np.degrees(THETA_MAX):.0f}°])')

    # Outlier removal: per-r-slice Tukey IQR fence on phi 
    TUKEY_CONSTANT = 1.5
    keep = np.ones(len(r), dtype=bool)
    for r_val in r_vals:
        idx = np.where(r == r_val)[0]
        if len(idx) < 5:
            continue
        p = phi_all[idx]
        q1, q3 = np.percentile(p, [25, 75])
        iqr = q3 - q1
        keep[idx] = (p >= q1 - TUKEY_CONSTANT * iqr) & (p <= q3 + TUKEY_CONSTANT * iqr)
    n_outliers = (~keep).sum()
    r, vf, vl = r[keep], vf[keep], vl[keep]
    theta_c, phi_c = theta_all[keep], phi_all[keep]
    print(f'{name} -- After outlier removal: {len(r)} points ({n_outliers} dropped)')

    # Report valid domain
    bounds = compute_valid_bounds(r, vf, vl)
    g = bounds['global']
    print(f"{name} -- Valid domain:  r=[{g['r'][0]:.2f}, {g['r'][1]:.2f}]  "
          f"vf=[{g['vf'][0]:.2f}, {g['vf'][1]:.2f}]  "
          f"vl=[{g['vl'][0]:.2f}, {g['vl'][1]:.2f}]")

    # Fit
    def poly_fit(X, y):
        model = Pipeline([
            ('poly', PolynomialFeatures(poly_degree)),
            ('reg', LinearRegression()),
        ])
        model.fit(X, y)
        return model, r2_score(y, model.predict(X))

    # phi(r, vf, vl) — vl is a meaningful predictor for phi
    phi_model, r2_phi = poly_fit(np.column_stack([r, vf, vl]), phi_c)
    # theta(r, vl)  — theta is essentially independent of vf
    theta_model, r2_theta = poly_fit(np.column_stack([r, vl]), theta_c)

    print(f'{name} -- phi R2 = {r2_phi:.6f} (inputs: r, vf, vl)')
    print(f'{name} -- theta R2 = {r2_theta:.6f} (inputs: r, vl)\n')

    return phi_model, theta_model, bounds


def export_model(model, path):
    poly = model.named_steps['poly']
    reg  = model.named_steps['reg']
    json.dump({
        'intercept': reg.intercept_,
        'coefs': reg.coef_.tolist(),
        'powers': poly.powers_.tolist(),
    }, open(path, 'w'), indent=2)


base = 'ProjectileMotionSim/TrajectorySurfaces'
dir_list = ['30_RPS_T', '34_RPS_T', '40_RPS_T']

for dir in dir_list:
    phi_model, theta_model, bounds = fit_surfaces(
        f'{base}/{dir}/clean_theta_phi_surface.npz', dir
    )

    os.makedirs(f'{base}/{dir}/models', exist_ok=True)

    with open(f'{base}/{dir}/models/phi_model.pkl', 'wb') as f:
        pickle.dump(phi_model, f)
    with open(f'{base}/{dir}/models/theta_model.pkl', 'wb') as f:
        pickle.dump(theta_model, f)

    export_model(phi_model, f'{base}/{dir}/models/phi_model.json')
    export_model(theta_model, f'{base}/{dir}/models/theta_model.json')

    # Save bounds (includes angle limits) for inference-time range checking
    json.dump(bounds, open(f'{base}/{dir}/models/valid_bounds.json', 'w'), indent=2)

    print(f'{dir} -- Models and bounds saved.\n')
    