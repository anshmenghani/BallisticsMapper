import numpy as np
import matplotlib.pyplot as plt
import pickle
import os

# Valid angle bounds — must match FitTrajectorySurface.py
PHI_MIN, PHI_MAX = np.radians(45), np.radians(75)   # launch elevation
THETA_MIN, THETA_MAX = np.radians(0), np.radians(340)  # azimuth


def mask_to_nan(surface, lo, hi):
    #Return a copy of `surface` with values outside [lo, hi] set to NaN.
    out = surface.astype(float).copy()
    out[(out < lo) | (out > hi)] = np.nan
    return out


def plot_save_models(name):
    base = f"ProjectileMotionSim/TrajectorySurfaces/{name}"
    os.makedirs(base + "/plots", exist_ok=True)

    # load data
    data = np.load(base + "/clean_theta_phi_surface.npz")
    r_vals  = data['r_vals']
    vf_vals = data['vf_vals']
    vl_vals = data['vl_vals']
    theta_surface = data['theta_surface']
    phi_surface = data['phi_surface']

    # load models
    with open(base + '/models/phi_model.pkl', 'rb') as f:
        phi_model = pickle.load(f)
    with open(base + '/models/theta_model.pkl', 'rb') as f:
        theta_model = pickle.load(f)

    # Grids
    zero_vf_idx = np.argmin(np.abs(vf_vals))
    zero_vl_idx = np.argmin(np.abs(vl_vals))

    R_t, VL_t = np.meshgrid(r_vals, vl_vals, indexing='ij')
    R_p, VF_p = np.meshgrid(r_vals, vf_vals, indexing='ij')

    # Raw data slices — out-of-bounds values set to NaN so they vanish
    theta_slice = mask_to_nan(theta_surface[:, zero_vf_idx, :], THETA_MIN, THETA_MAX)
    phi_slice   = mask_to_nan(phi_surface[:,  :,  zero_vl_idx], PHI_MIN,   PHI_MAX)

    # Model predictions — same clamping applied
    theta_fit = mask_to_nan(
        theta_model.predict(
            np.column_stack([R_t.ravel(), VL_t.ravel()])
        ).reshape(R_t.shape),
        THETA_MIN, THETA_MAX
    )

    vl_zero = np.zeros(R_p.size)
    phi_fit = mask_to_nan(
        phi_model.predict(
            np.column_stack([R_p.ravel(), VF_p.ravel(), vl_zero])
        ).reshape(R_p.shape),
        PHI_MIN, PHI_MAX
    )

    # Consistent z-limits per angle (span of valid data only)
    theta_zlim = (np.nanmin(theta_slice), np.nanmax(theta_slice))
    phi_zlim = (np.nanmin(phi_slice), np.nanmax(phi_slice))

    def fmt_theta(ax, title):
        ax.set_xlabel('r (m)'); ax.set_ylabel('vl (m/s)'); ax.set_zlabel('θ (rad)')
        ax.set_zlim(*theta_zlim)
        ax.set_title(f'{title}\nθ valid: [{np.degrees(THETA_MIN):.0f}°–{np.degrees(THETA_MAX):.0f}°]')

    def fmt_phi(ax, title):
        ax.set_xlabel('r (m)'); ax.set_ylabel('vf (m/s)'); ax.set_zlabel('φ (rad)')
        ax.set_zlim(*phi_zlim)
        ax.set_title(f'{title}\nφ valid: [{np.degrees(PHI_MIN):.0f}°–{np.degrees(PHI_MAX):.0f}°]')

    # Plot 1: raw theta surface
    fig1, ax1 = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(10, 6))
    surf1 = ax1.plot_surface(R_t, VL_t, theta_slice, cmap='viridis', edgecolor='k')
    fmt_theta(ax1, f'Theta Surface – {name}  (vf=0 slice)')
    fig1.colorbar(surf1, ax=ax1, shrink=0.5, aspect=10)
    fig1.savefig(base + "/plots/theta_surface.png", dpi=300, bbox_inches='tight')

    # Plot 2: raw phi surface
    fig2, ax2 = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(10, 6))
    surf2 = ax2.plot_surface(R_p, VF_p, phi_slice, cmap='plasma', edgecolor='k')
    fmt_phi(ax2, f'Phi Surface – {name}  (vl=0 slice)')
    fig2.colorbar(surf2, ax=ax2, shrink=0.5, aspect=10)
    fig2.savefig(base + "/plots/phi_surface.png", dpi=300, bbox_inches='tight')

    # Plot 3: theta data + fit overlay
    fig3, ax3 = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(10, 6))
    ax3.plot_surface(R_t, VL_t, theta_slice, cmap='viridis', alpha=0.6, edgecolor='none')
    ax3.plot_surface(R_t, VL_t, theta_fit,   cmap='cool',    alpha=0.6, edgecolor='none')
    fmt_theta(ax3, f'Theta: Data (viridis) vs Fit (cool) – {name}')
    fig3.savefig(base + "/plots/theta_overlay.png", dpi=300, bbox_inches='tight')

    # Plot 4: theta fit only
    fig4, ax4 = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(10, 6))
    surf4 = ax4.plot_surface(R_t, VL_t, theta_fit, cmap='viridis', edgecolor='k')
    fmt_theta(ax4, f'Theta Polynomial Fit – {name}')
    fig4.colorbar(surf4, ax=ax4, shrink=0.5, aspect=10)
    fig4.savefig(base + "/plots/theta_fit.png", dpi=300, bbox_inches='tight')

    # Plot 5: phi data + fit overlay
    fig5, ax5 = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(10, 6))
    ax5.plot_surface(R_p, VF_p, phi_slice, cmap='plasma', alpha=0.6, edgecolor='none')
    ax5.plot_surface(R_p, VF_p, phi_fit,   cmap='cool',   alpha=0.6, edgecolor='none')
    fmt_phi(ax5, f'Phi: Data (plasma) vs Fit (cool) – {name}  (vl=0)')
    fig5.savefig(base + "/plots/phi_overlay.png", dpi=300, bbox_inches='tight')

    # Plot 6: phi fit only
    fig6, ax6 = plt.subplots(subplot_kw={'projection': '3d'}, figsize=(10, 6))
    surf6 = ax6.plot_surface(R_p, VF_p, phi_fit, cmap='plasma', edgecolor='k')
    fmt_phi(ax6, f'Phi Polynomial Fit – {name}  (vl=0)')
    fig6.colorbar(surf6, ax=ax6, shrink=0.5, aspect=10)
    fig6.savefig(base + "/plots/phi_fit.png", dpi=300, bbox_inches='tight')

    plt.show()
    plt.close('all')
    print(f"{name} -- plots saved.")


dir_list = ["40_RPS", "50_RPS", "60_RPS"]

for d in dir_list:
    plot_save_models(d)
    