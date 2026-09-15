# Projectile Motion Simulator

A physics-based projectile simulation and inverse-ballistics solver developed for the **FRC 900 Zebracorns' 2026 Fuel shooter**.

The project models a launched Fuel using numerical integration and solves the inverse problem:

> Given the robot's position and velocity relative to the Hub, what launch direction is required for the Fuel to reach the target?

The simulator accounts for **gravity, aerodynamic drag, Magnus force from backspin, and rotational drag**. An inverse solver based on the **Levenberg–Marquardt algorithm** determines the launch angles required to hit a specified target.

The project also generates precomputed trajectory surfaces and polynomial approximations so that the computationally expensive physics simulation can eventually be replaced by fast real-time calculations on the robot.

---

## 1. Project Overview

There are three main stages to the project:

```text
Physics Model
    │
    ▼
ProjectilePath.py
    │
    │  Runge-Kutta 4 simulation
    │  + Levenberg-Marquardt inverse solver
    ▼
TrajectorySurface.py
    │
    │  Solve thousands of trajectories
    ▼
clean_theta_phi_surface.npz
    │
    ▼
FitTrajectorySurface.py
    │
    │  Polynomial regression
    ▼
phi_model / theta_model
    │
    ├── Python (.pkl / .json)
    │
    └── Java (PolynomialModel.java)
```

At runtime, the intended workflow is approximately:

```text
Robot distance (r)
Robot radial velocity (vf)
Robot tangential velocity (vl)
Shooter RPS
        │
        ▼
Precomputed polynomial model
        │
        ▼
Required launch angles
        │
        ▼
Turret / hood / shooter control
```

The precomputed models are especially useful because running the full RK4 + Levenberg–Marquardt solver for every control-loop iteration would be far too expensive.

---

# 2. Coordinate System

Understanding the coordinate system is probably the most important thing for a new contributor.

The physics simulator uses:

* **X** — horizontal direction toward the target
* **Y** — vertical direction
* **Z** — horizontal direction perpendicular to X

So the projectile position is:

```text
(x, y, z)
```

with gravity acting in the **-Y direction**.

For a shot directly toward the Hub:

```text
          Y
          ↑
          │
          │       • Hub
          │      /
          │     /
          │    /  projectile
          │   /
          └────────────────→ X
         /
        /
       Z
```

The target is generally represented as:

```python
xt = r
yt = hub_height
zt = 0
```

where `r` is the horizontal distance to the Hub.

The robot's horizontal velocity is decomposed into:

* `vf` — **forward/radial velocity**, toward or away from the target
* `vl` — **lateral/tangential velocity**, perpendicular to the target direction

In the simulator, these are ultimately applied as frame velocities:

```python
vx_frame = vf
vz_frame = vl
```

while the robot's vertical velocity is normally zero.

### Important distinction: launch velocity vs. robot velocity

The solver's `vx`, `vy`, and `vz` describe the projectile's velocity **relative to the robot/shooter**.

The frame velocities describe the robot's motion.

The actual projectile velocity used in the simulation is therefore approximately:

```text
vx_total = vx + vx_frame
vy_total = vy + vy_frame
vz_total = vz + vz_frame
```

This distinction is critical when modifying the solver.

---

# 3. `phi` vs. `theta`: Important Naming Inconsistency

There is currently an inconsistency in the naming of the two launch angles between different files.

### In `ProjectilePath.py`

The angles are defined mathematically as:

```python
theta = arcsin(vz / v_mag)
phi   = arctan2(vy, vx)
```

Thus:

* **`theta`** = angle involving the Z component / vertical elevation in the simulator's mathematical definition
* **`phi`** = azimuth in the X-Y plane according to the comments

However, the trajectory-surface code uses the names differently.

### In `FitTrajectorySurface.py`

The project explicitly documents:

```text
phi   : launch elevation
theta : azimuth
```

and uses:

```text
phi(r, vf, vl)
theta(r, vl)
```

The plotting code follows the same convention:

```text
φ = launch elevation
θ = azimuth
```

This is the convention used by the **trajectory surfaces and fitted models**.

### What should a new user assume?

For the purposes of the **trajectory-surface/model pipeline**, use:

| Name    | Meaning in surface pipeline   |
| ------- | ----------------------------- |
| `phi`   | Launch elevation / hood angle |
| `theta` | Azimuth / turret angle        |

This is the convention used in:

* `TrajectorySurface.py`
* `FitTrajectorySurface.py`
* `PlotTrajectorySurfaces.py`
* `GenerateValidShotProfile.py`
* generated CSV/NPZ files
* polynomial models

However, `ProjectilePath.py` contains older/different mathematical naming.

**Do not rename variables casually without checking every downstream file.** The model files and CSV columns currently use `phi` and `theta` according to the surface convention.

---

# 4. Core Physics — `ProjectilePath.py`

`ProjectilePath.py` contains the fundamental physics model.

## `Projectile`

The `Projectile` class represents the Fuel and contains the aerodynamic model.

The current FRC Fuel is initialized approximately as:

```python
fuel = Projectile(
    radius=0.0762,
    mass=0.226796
)
```

The model includes:

### Gravity

Gravity acts downward:

```text
a_y = -g
```

with:

```python
g = 9.81
```

### Drag

Aerodynamic drag opposes the projectile's velocity.

The drag coefficient is modeled as a function of Reynolds number:

```python
drag_coeff = lambda Re: 0.47
```

by default.

### Magnus force

Backspin generates a lift force through the Magnus effect.

The default lift model is:

```python
lift_coeff = lambda S: 1.5 * S
```

where `S` is the shear parameter.

### Rotational drag

The projectile's spin decreases over time according to the rotational-drag model.

The simulator assumes spin about the **Z axis** and uses:

```python
n_hat = +1
```

for backspin.

---

# 5. Runge-Kutta 4 Simulation

`RungeKutta4` numerically integrates the projectile's differential equations.

The simulation advances in discrete timesteps:

```python
dt = 0.01
```

meaning the default timestep is **10 ms**.

The velocity equations are integrated first. Position is then obtained by integrating the resulting velocity.

The important method is:

```python
Projectile.trajectory(...)
```

It returns velocity, position, final speed, and final angular velocity information.

For example:

```python
results = fuel.trajectory(
    vx0,
    vy0,
    vz0,
    omega0=omega,
    dt=0.01
)
```

The return values are:

```text
velocity time list
vx list
vy list
vz list
final speed
position time list
x position list
y position list
z position list
final angular velocity
```

---

# 6. Inverse Ballistics — `ProjectileSolver`

Normally, the physics simulation answers:

> "If I launch the Fuel with this velocity, where will it go?"

`ProjectileSolver` answers the opposite:

> "What velocity/angles should I launch with to reach this location?"

The solver starts with an initial guess and repeatedly adjusts its parameters.

The active parameters can include:

```text
v_mag
theta
phi
omega
```

depending on whether speed and spin are fixed.

For the normal shooter use case, speed and spin are fixed:

```python
fix_speed=True
fix_omega=True
```

so the solver primarily changes the launch angles.

---

# 7. Levenberg–Marquardt Solver

The inverse solver uses the **Levenberg–Marquardt algorithm**.

Conceptually:

```text
Initial guess
     │
     ▼
Simulate projectile
     │
     ▼
Compare final position to target
     │
     ▼
Calculate Jacobian
     │
     ▼
Adjust launch parameters
     │
     ▼
Repeat
```

The solver minimizes the residual between the desired and simulated final state.

The residual contains:

```text
X position error
Y position error
Z position error
clearance error
```

The `clearance_func` allows additional physical constraints to be incorporated into the optimization.

---

# 8. Hub Clearance — `FuelClearance.py`

`FuelClearance.py` implements the Hub-clearance constraint used by the solver.

The idea is that a trajectory should not simply reach the Hub center; it must also clear the physical edge of the Hub appropriately.

The function:

```python
hub_clearance(...)
```

examines the trajectory near the Hub and calculates a penalty if the Fuel is below the required clearance height.

This penalty is passed into the Levenberg–Marquardt residual.

A new user should therefore be aware that the solver is **not simply solving three independent position equations**. It is also optimizing against the clearance constraint.

---

# 9. Shooter Model

`TrajectorySurface.py` contains the FRC 900-specific relationship between shooter RPS, projectile speed, and spin.

The current model is:

```python
speed_in_rps = abs((1 - 0.233333333) * rps)
speed = speed_in_rps * 0.31
```

and:

```python
spin_in_rps = abs(0.233333333 * rps)
spin = spin_in_rps * 2π
```

Therefore, changing the shooter RPS changes **both projectile speed and backspin**.

This is important: RPS is not merely a projectile-speed parameter in this model.

If the shooter hardware changes, these constants will likely need to be recalibrated.

---

# 10. Generating Trajectory Surfaces

`TrajectorySurface.py` generates a large lookup surface by repeatedly running the inverse solver.

For every combination of:

```text
r
vf
vl
```

the program attempts to determine the launch angles needed for the shot.

The current grid is approximately:

```text
r  : 0 → 6.5 m
vf : -5.5 → +5.5 m/s
vl : -5.5 → +5.5 m/s
```

The code skips combinations where:

```python
hypot(vf, vl) > 5.5
```

because they exceed the intended robot-speed domain.

Each successful run stores:

```text
theta
phi
```

in:

```text
clean_theta_phi_surface.npz
```

### Warning

This is the **computationally expensive part of the project**.

Generating an entire surface requires thousands of individual projectile simulations and inverse solves. It should not be necessary during normal robot operation.

---

# 11. Trajectory Surface Files

Each RPS directory contains a precomputed surface:

```text
TrajectorySurfaces/
├── 40_RPS/
├── 50_RPS/
└── 60_RPS/
```

Each contains:

```text
clean_theta_phi_surface.npz
models/
plots/
```

The `.npz` file contains:

```text
r_vals
vf_vals
vl_vals
theta_surface
phi_surface
```

The surface arrays have dimensions corresponding to:

```text
[r, vf, vl]
```

Therefore:

```python
theta_surface[i, j, k]
```

corresponds to:

```text
r_vals[i]
vf_vals[j]
vl_vals[k]
```

and similarly for `phi_surface`.

---

# 12. Fitting Polynomial Models

`FitTrajectorySurface.py` converts the expensive simulation results into fast polynomial models.

The project uses scikit-learn's:

```python
PolynomialFeatures
LinearRegression
```

with a default polynomial degree of 4.

The fitted models are:

### Elevation

```text
phi = f(r, vf, vl)
```

All three variables can affect the required elevation.

### Azimuth

```text
theta = f(r, vl)
```

The model intentionally does not include `vf` because azimuth is treated as essentially independent of radial velocity.

This reflects the physical interpretation:

* **Distance + radial velocity** primarily affect the vertical/elevation solution.
* **Tangential velocity** causes the projectile to need an azimuthal correction.

---

# 13. Polynomial Model Outputs

For each RPS setting, fitting produces:

```text
models/
├── phi_model.pkl
├── phi_model.json
├── theta_model.pkl
├── theta_model.json
└── valid_bounds.json
```

### `.pkl`

Python/scikit-learn version of the fitted model.

Useful when working entirely in Python.

### `.json`

Portable representation of the polynomial.

It contains:

```json
{
    "intercept": ...,
    "coefs": [...],
    "powers": [...]
}
```

This is intended to make the model usable outside Python.

### `valid_bounds.json`

Contains the domain over which the model was trained, along with the valid mechanical angle ranges.

**Do not assume the polynomial is reliable outside this domain.**

Polynomial regression can produce plausible-looking but physically meaningless results when extrapolated.

---

# 14. Java Integration

`LaunchJavaFiles/PolynomialModel.java` loads the exported JSON model and evaluates it directly in Java.

The basic interface is:

```java
PolynomialModel model = PolynomialModel.load("phi_model.json");

double output = model.evaluate(x, y);
```

The JSON polynomial is evaluated as:

```text
intercept + Σ(coefficient × x^power × y^power)
```

This avoids requiring Python/scikit-learn on the robot.

For the current models:

```text
phi_model
    → inputs: r, vf, vl

theta_model
    → inputs: r, vl
```

The Java implementation therefore needs to know **which model it is loading** so that the correct inputs are passed.

---

# 15. Valid Shot Tables

`GenerateValidShotProfile.py` combines the precomputed surfaces from multiple shooter RPS settings.

The current configuration considers:

```text
40 RPS
50 RPS
60 RPS
```

and keeps combinations where both angles are mechanically achievable.

The valid angle ranges are:

```text
phi   = 45° → 75°
theta = 0° → 340°
```

The resulting table contains:

```text
r
vf
vl
rps
phi_rad
theta_rad
phi_deg
theta_deg
```

A single `(r, vf, vl)` combination can have multiple valid RPS values.

That means the table is not necessarily a one-to-one mapping from robot state to shooter setting.

A higher-level controller can choose between the available RPS solutions according to its own criteria.

---

# 16. Verification — `FuelPath.py`

`FuelPath.py` provides a useful sanity check.

It:

1. Loads valid trajectories.
2. Randomly selects a number of them.
3. Runs the physics solver again.
4. Simulates the resulting shot.
5. Checks how close the projectile lands to the target.
6. Reports angle errors and landing errors.

The random seed is fixed:

```python
RANDOM_SEED = 42
```

so the same samples can be reproduced.

This is useful when modifying the physics model or surface-generation pipeline.

---

# 17. Plotting Results

`PlotTrajectorySurfaces.py` creates visualizations of the raw trajectory surfaces and polynomial fits.

For each RPS setting it produces plots such as:

```text
theta_surface.png
phi_surface.png

theta_overlay.png
phi_overlay.png

theta_fit.png
phi_fit.png
```

The overlay plots are particularly useful for checking whether the polynomial approximation follows the original physics simulation.

When evaluating a new model, do not rely only on the regression `R²`. A model can have a high global `R²` while still behaving poorly near important operating boundaries.

---

# 18. Recommended Setup

The project is Python-based and uses packages including:

```text
numpy
matplotlib
scikit-learn
pandas
tabulate
```

A clean Python virtual environment is recommended.

From the project root:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Then install the required packages:

```bash
pip install numpy matplotlib scikit-learn pandas tabulate
```

On Windows, activation is:

```powershell
.venv\Scripts\activate
```

The repository currently contains a `.venv` directory, but **do not rely on the checked-in environment**. A new contributor should create their own environment rather than using someone else's Python environment.

---

# 19. Running the Existing Models

If you only want to inspect or use the existing results, start with:

```text
TrajectorySurfaces/
```

The precomputed `.npz`, model, and plot files are already present.

For Python inference, load the `.pkl` models:

```python
import pickle

with open("TrajectorySurfaces/50_RPS/models/phi_model.pkl", "rb") as f:
    phi_model = pickle.load(f)

with open("TrajectorySurfaces/50_RPS/models/theta_model.pkl", "rb") as f:
    theta_model = pickle.load(f)
```

Then:

```python
phi = phi_model.predict([[r, vf, vl]])[0]
theta = theta_model.predict([[r, vl]])[0]
```

Remember that these outputs are in **radians**.

Convert to degrees with:

```python
import numpy as np

phi_deg = np.degrees(phi)
theta_deg = np.degrees(theta)
```

---

# 20. Rebuilding the Models

If you change the physics model, shooter calibration, angle limits, or operating range, the precomputed models should be regenerated.

The conceptual pipeline is:

### Step 1 — Generate surfaces

Run the appropriate configuration in:

```text
TrajectorySurfaces/TrajectorySurface.py
```

This creates:

```text
clean_theta_phi_surface.npz
```

### Step 2 — Fit surfaces

Run:

```text
TrajectorySurfaces/FitTrajectorySurface.py
```

This produces the `.pkl`, `.json`, and bounds files.

### Step 3 — Generate valid shot table

Run:

```text
TrajectorySurfaces/GenerateValidShotProfile.py
```

if the valid RPS lookup table is needed.

### Step 4 — Plot and inspect

Run:

```text
TrajectorySurfaces/PlotTrajectorySurfaces.py
```

to visually compare the fitted models against the simulated data.

### Step 5 — Verify

Run:

```text
FuelPath.py
```

to test randomly selected solutions against the full physics simulation.

---

# 21. Important Configuration Values

Several values are currently hard-coded throughout the project.

These include:

* Fuel radius
* Fuel mass
* Hub height
* Shooter height
* Air density
* Drag coefficient
* Lift coefficient
* Shooter speed calibration
* Shooter spin calibration
* Valid `phi` range
* Valid `theta` range
* Distance range
* Robot velocity range
* RK4 timestep
* Levenberg–Marquardt iteration count

If one of these values changes, **search the entire project for the old value** before assuming that changing it in one file is sufficient.

For example, the Hub height and launch height are defined in multiple scripts.

This duplication is a potential source of bugs.

---

# 22. Common Points of Confusion

## `phi` and `theta`

As described above, the angle naming is inconsistent between `ProjectilePath.py` and the trajectory-surface pipeline.

For the generated surface/model files:

```text
phi   = elevation
theta = azimuth
```

Use this convention when interacting with the lookup tables and polynomial models.

---

## `vf` and `vl`

These are **not Cartesian X and Z coordinates**.

They describe robot velocity relative to the target:

```text
vf = forward/radial velocity
vl = lateral/tangential velocity
```

They are eventually mapped into the simulator's coordinate system.

---

## RPS vs. rad/s

Shooter RPS and projectile spin are different quantities.

The shooter configuration is specified in:

```text
RPS
```

while angular velocity used by the physics model is:

```text
rad/s
```

The conversion is approximately:

```python
rad_s = RPS * 2π
```

after accounting for the fraction of shooter motion contributing to ball spin.

---

## RPS vs. projectile speed

The shooter RPS is not directly equal to projectile speed.

The project currently uses an empirical conversion involving:

```python
0.233333333
```

and:

```python
0.31
```

These are shooter-specific calibration constants.

---

## Degrees vs. radians

Internally, angles are stored in **radians**.

The valid ranges are written in degrees for readability:

```text
phi:   45°–75°
theta: 0°–340°
```

but code generally uses:

```python
np.radians(...)
```

and model outputs are radians.

---

## The model is not an arbitrary mathematical fit

The polynomial models are approximations of the physics simulation.

They should therefore be treated as **surrogate models**, not replacements for the underlying physics.

If the underlying projectile model changes significantly, the fitted models need to be regenerated.

---

## Valid bounds matter

A polynomial can return a number for an input that was never present in the training data.

That does **not** mean the result is valid.

Always check the input against:

```text
valid_bounds.json
```

before relying on a model prediction.

---

# 23. Suggested Workflow for a New Contributor

If you are new to the project, read the files in this order:

### 1. `ProjectilePath.py`

Understand:

* coordinate system
* projectile state
* RK4 integration
* drag
* Magnus force
* rotational drag
* inverse solver

### 2. `FuelClearance.py`

Understand how the Hub constraint is incorporated into the solver.

### 3. `TrajectorySurface.py`

Understand how the physics solver is repeatedly used to create a lookup surface.

### 4. `FitTrajectorySurface.py`

Understand how the expensive simulation is converted into fast polynomial models.

### 5. `PlotTrajectorySurfaces.py`

Use this to visually understand what the models look like.

### 6. `GenerateValidShotProfile.py`

Understand how valid shooter settings are determined.

### 7. `FuelPath.py`

Use this to see how the generated solutions are independently verified.

### 8. `LaunchJavaFiles/PolynomialModel.java`

Read this when integrating the fitted models into Java/robot code.

---

# 24. High-Level Mental Model

The easiest way to think about the entire project is:

```text
                 PHYSICS
                   │
                   ▼
        "Where will this Fuel go?"
                   │
                   │ RK4
                   ▼
             Trajectory
                   │
                   ▼
              INVERSE SOLVER
                   │
                   │ Levenberg–Marquardt
                   ▼
       "What angles hit the Hub?"
                   │
                   ▼
          TRAJECTORY SURFACES
                   │
                   │ thousands of solutions
                   ▼
             POLYNOMIAL FIT
                   │
                   ▼
         FAST REAL-TIME MODEL
                   │
                   ▼
       r, vf, vl ──► angles
                   │
                   ▼
             ROBOT CONTROL
```

The fundamental goal is therefore not simply to simulate projectile motion. It is to turn a computationally expensive physics problem into a **fast, deployable ballistics model** that can provide shooting solutions from the robot's current distance and velocity.

---

# 25. Known Technical Caveats

A few things should be kept in mind when extending the project:

1. **Angle naming is inconsistent.**
   The surface pipeline uses `phi = elevation` and `theta = azimuth`, while `ProjectilePath.py` uses different mathematical definitions.

2. **Several physical constants are duplicated.**
   Changing a value in one file may not change it everywhere.

3. **The polynomial models are only trustworthy inside their training domain.**

4. **The fitted model's domain is not necessarily a rectangular region in practice.**
   `valid_bounds.json` contains useful bounds, but a simple global min/max check does not guarantee that every point inside the box has a valid trajectory.

5. **The surface generator is computationally expensive.**
   Do not regenerate surfaces unnecessarily.

6. **The shooter calibration is hardware-specific.**
   The RPS → speed/spin relationship and angle bounds should be recalibrated if the shooter changes.

7. **The physics model is an approximation.**
   Drag and lift coefficients, air properties, and shooter calibration all affect the final accuracy.

8. **The generated files represent a particular version of the physics model.**
   If `ProjectilePath.py` changes, previously generated surfaces/models may no longer correspond to the source physics.

---

## 26. File Structure

A simplified view of the repository is:

```text
ProjectileMotionSim/
│
├── ProjectilePath.py              # Core physics + RK4 + inverse solver
├── FuelClearance.py               # Hub clearance constraint
├── FuelPath.py                    # Verification/testing
├── BallisticsMapper.pdf           # Supporting documentation
│
├── TrajectorySurfaces/
│   ├── TrajectorySurface.py       # Generate simulated surfaces
│   ├── FitTrajectorySurface.py    # Fit polynomial surrogate models
│   ├── PlotTrajectorySurfaces.py  # Visualize surfaces/models
│   ├── GenerateValidShotProfile.py
│   ├── LoadSurface.py             # Legacy CSV joining utility
│   │
│   ├── 40_RPS/
│   ├── 50_RPS/
│   └── 60_RPS/
│
├── LaunchJavaFiles/
│   └── PolynomialModel.java       # Java polynomial evaluator
│
└── RK4ExampleOutputs/             # Example simulation outputs
```

There are also legacy/generated files in the repository. In particular, `LoadSurface.py` uses the older `valid_phi.csv` / `valid_theta.csv` workflow, while `GenerateValidShotProfile.py` produces the newer unified `valid_shots.csv` / `valid_shots.npz` table.

If you are extending the project, prefer understanding the newer trajectory-surface pipeline before modifying the older CSV utilities.

---

## 27. In Short

If you only remember five things:

1. **`ProjectilePath.py` is the physics engine.**
2. **`ProjectileSolver` solves the inverse ballistics problem using Levenberg–Marquardt.**
3. **`TrajectorySurface.py` precomputes solutions over distance and robot velocity.**
4. **`FitTrajectorySurface.py` turns those expensive solutions into fast polynomial models.**
5. **For the surface/model pipeline, `phi` means elevation and `theta` means azimuth, despite the older naming in `ProjectilePath.py`.**

When in doubt, trace the data flow:

```text
(r, vf, vl, RPS)
        ↓
 projectile speed + spin
        ↓
 physics simulation
        ↓
 inverse solution
        ↓
 phi/elevation + theta/azimuth
        ↓
 polynomial model
        ↓
 Java / robot control
```
