import csv

KEY = ('r', 'vf', 'vl', 'rps')

# load phi
phi_map = {}
with open('ProjectileMotionSim/TrajectorySurfaces/valid_phi.csv') as f:
    for row in csv.DictReader(f):
        key = (float(row['r']), float(row['vf']), float(row['vl']), int(row['rps']))
        phi_map[key] = {'phi_rad': row['phi_rad'], 'phi_deg': row['phi_deg']}

# load theta
theta_map = {}
with open('ProjectileMotionSim/TrajectorySurfaces/valid_theta.csv') as f:
    for row in csv.DictReader(f):
        key = (float(row['r']), float(row['vf']), float(row['vl']), int(row['rps']))
        theta_map[key] = {'theta_rad': row['theta_rad'], 'theta_deg': row['theta_deg']}

# inner join
matched_keys = phi_map.keys() & theta_map.keys()

rows = []
for key in sorted(matched_keys):
    r, vf, vl, rps = key
    rows.append({
        'r': r,
        'vf': vf,
        'vl': vl,
        'rps': rps,
        **phi_map[key],
        **theta_map[key],
    })

print(f'phi points: {len(phi_map)}')
print(f'theta points: {len(theta_map)}')
print(f'Matched: {len(rows)}')
print(f'Unmatched phi-only:   {len(phi_map) - len(rows)}')
print(f'Unmatched theta-only: {len(theta_map) - len(rows)}')

with open('valid_trajectories.csv', 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=['r', 'vf', 'vl', 'rps', 'phi_rad', 'phi_deg', 'theta_rad', 'theta_deg'])
    writer.writeheader()
    writer.writerows(rows)

print('Saved to valid_trajectories.csv')
