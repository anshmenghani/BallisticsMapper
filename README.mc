# BallisticsMapper / Projectile Motion Simulator

A physics-based projectile simulator and inverse-ballistics pipeline developed for the FRC 900 Zebracorns' 2026 Fuel shooter.

The project has two main purposes:

1. **Simulate a Fuel trajectory** using a physics model containing gravity, aerodynamic drag, Magnus lift from backspin, and rotational drag.
2. **Solve the inverse problem:** given the robot's distance and velocity relative to the Hub, determine the launch angles needed to make the Fuel reach the target.

Because solving the full physics problem numerically is too expensive to perform continuously on the robot, the project also builds **precomputed trajectory surfaces** and fits them with polynomial models. These models provide a fast approximation suitable for real-time robot code.

---

# 1. The Big Picture

The complete pipeline is:

```text
                         ┌──────────────────────┐
                         │   Projectile Physics │
                         │    ProjectilePath.py │
                         └──────────┬───────────┘
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │       RK4 Solver     │
                         │  Forward simulation  │
                         └──────────┬───────────┘
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │  Levenberg-Marquardt │
                         │    Inverse Solver    │
                         └──────────┬───────────┘
                                    │
                     solve thousands of states
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │ Trajectory Surfaces  │
                         │   (r, vf, vl) →      │
                         │      angles         │
                         └──────────┬───────────┘
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │  Data Cleaning       │
                         │  angle filtering     │
                         │  Tukey IQR filtering │
                         └──────────┬───────────┘
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │ Polynomial Regression│
                         │      degree = 4      │
                         └──────────┬───────────┘
                                    │
                         ┌──────────┴──────────┐
                         ▼                     ▼
                  Python .pkl/.json       Java evaluator
                         │                     │
                         └──────────┬──────────┘
                                    ▼
                             Robot-side model

```

The important conceptual transformation is:

```text
Full physics:
(r, vf, vl, shooter setting)
             ↓
       numerical solver
             ↓
       launch angles

Offline approximation:
(r, vf, vl, shooter setting)
             ↓
      polynomial model
             ↓
       launch angles

```

The polynomial model is therefore a **surrogate for the physics/inverse solver**, not an independent physics model.

---

# 2. What Problem Does This Solve?

For the FRC shooter, the desired behavior is:

> Given the robot's horizontal distance from the Hub and its velocity relative to the Hub, determine the projectile launch direction required to score.

The relevant robot-state variables are:

* `r` — horizontal/radial distance to the Hub
* `vf` — forward/radial velocity relative to the Hub
* `vl` — lateral/tangential velocity relative to the Hub
* `RPS` — shooter speed

The robot's velocity matters because the Fuel inherits the robot's motion.

For example:

```text
Robot moving directly toward Hub
        ───────────────►

                Hub
                 ○

```

The projectile already has forward velocity from the robot, so the shooter does not need to provide as much forward projectile velocity.

Conversely, if the robot is moving sideways:

```text
             Hub
              ○
              ↑
              │
              │
              │
Robot ────────►

```

the projectile must be launched at an azimuth that compensates for the robot's tangential velocity.

This is why the model depends on both **radial velocity** and **tangential velocity**.

---

# 3. Repository Structure

The important files are:

```text
ProjectileMotionSim/
│
├── ProjectilePath.py
├── FuelClearance.py
├── FuelPath.py
│
├── BallisticsMapper.pdf
│
├── TrajectorySurfaces/
│   ├── TrajectorySurface.py
│   ├── FitTrajectorySurface.py
│   ├── GenerateValidShotProfile.py
│   ├── PlotTrajectorySurfaces.py
│   ├── LoadSurface.py
│   │
│   ├── valid_phi.csv
│   ├── valid_theta.csv
│   ├── valid_trajectories.csv
│   ├── valid_trajectories_mirrored.csv
│   │
│   ├── valid_shot_table/
│   │   ├── valid_shots.csv
│   │   └── valid_shots.npz
│   │
│   ├── 40_RPS/
│   ├── 50_RPS/
│   └── 60_RPS/
│
├── LaunchJavaFiles/
│   └── PolynomialModel.java
│
└── RK4ExampleOutputs/

```

The `40_RPS`, `50_RPS`, and `60_RPS` directories contain already-generated data.

Each RPS directory contains approximately:

```text
40_RPS/
├── clean_theta_phi_surface.npz
├── models/
│   ├── phi_model.pkl
│   ├── phi_model.json
│   ├── theta_model.pkl
│   └── theta_model.json
└── plots/
    ├── phi_fit.png
    ├── phi_overlay.png
    ├── phi_surface.png
    ├── theta_fit.png
    ├── theta_overlay.png
    └── theta_surface.png

```

Some versions of the fitting pipeline also generate:

```text
models/valid_bounds.json

```

which stores the domain over which the fitted model was trained.

---

# 4. Setting Up Python

The project is Python-based.

The ZIP contains a `.venv`, but **a new user should not rely on the included virtual environment**. Virtual environments are machine-specific and generally should not be shared through Git.

Create a fresh environment:

```bash
python3 -m venv .venv
source .venv/bin/activate

```

On Windows:

```powershell
.venv\Scripts\activate

```

Install the major dependencies:

```bash
pip install numpy matplotlib scikit-learn pandas tabulate

```

The project also uses standard-library modules such as:

```text
json
pickle
csv
math
os
time
random
multiprocessing

```

---

# 5. `ProjectilePath.py` — The Physics Engine

`ProjectilePath.py` contains the fundamental projectile model.

It defines three important classes:

```text
RungeKutta4
Projectile
ProjectileSolver

```

---

## 5.1 `RungeKutta4`

`RungeKutta4` is a generic fourth-order Runge-Kutta numerical integrator.

The projectile equations are nonlinear, so they are integrated numerically rather than solved analytically.

The standard RK4 process evaluates the derivative four times per timestep:

```text
k1
k2
k3
k4

```

and combines them as:

```text
a(t + dt) ≈ a(t) + (k1 + 2k2 + 2k3 + k4)/6

```

The default timestep used throughout the project is:

```python
dt = 0.01

```

or 10 ms.

---

# 6. Projectile Physics

The `Projectile` class models the Fuel.

The default FRC Fuel parameters are approximately:

```text
radius = 0.0762 m
mass   = 0.226796 kg

```

The default atmospheric parameters are:

```text
air density     = 1.195 kg/m³
dynamic viscosity = 1.835e-5 N·s/m²
gravity         = 9.81 m/s²

```

The default aerodynamic parameters include:

```text
drag coefficient       = 0.47
rotational drag coeff. = 0.02
lift coefficient       = 1.5 × shear parameter

```

The moment of inertia defaults to:

```text
I = 0.4 m r²

```

unless explicitly provided.

---

# 7. Forces Included

The simulator accounts for four major effects.

## Gravity

Gravity acts in the negative Y direction:

```text
        +Y
         ↑
         │
         │
         ●
         ↓
         g

```

with:

```text
g = 9.81 m/s²

```

---

## Aerodynamic Drag

Drag opposes the projectile's velocity.

The simulator calculates the Reynolds number from:

```text
Re = ρ v (2r) / μ

```

and then uses the configured drag coefficient.

Currently:

```python
drag_coeff = lambda Re: 0.47

```

so the coefficient is effectively constant despite Reynolds number being calculated.

---

## Magnus Force

The Fuel is launched with backspin.

Backspin causes a Magnus/lift force that changes the trajectory.

The code calculates a shear parameter:

```text
S = |ω|r / v

```

and currently uses:

```python
lift_coeff = lambda S: 1.5 * S

```

The direction of the Magnus force depends on the spin direction.

The project represents:

```text
n_hat = +1 → backspin
n_hat = -1 → topspin
n_hat =  0 → no spin

```

---

## Rotational Drag

The projectile's spin decreases during flight.

The `omega()` function models this decay using the configured rotational drag coefficient.

Thus, the Fuel's spin is not treated as constant during the trajectory simulation even though the **initial spin can be fixed** during inverse solving.

---

# 8. Coordinate System

The simulator uses:

```text
x = horizontal direction toward target
y = vertical
z = horizontal direction perpendicular to target

```

So:

```text
                 y
                 ↑
                 │
                 │       Target
                 │          ●
                 │        /
                 │      /
                 │    /
                 └────────────────→ x
                /
               /
              z

```

The projectile velocity is:

```text
(vx, vy, vz)

```

The robot velocity is incorporated through:

```text
(vx_frame, vy_frame, vz_frame)

```

The total velocity actually used in the trajectory simulation is:

```text
vx_total = vx + vx_frame
vy_total = vy + vy_frame
vz_total = vz + vz_frame

```

This is one of the most important details in the project.

The solver determines the **shooter-provided velocity**, while the projectile simulation sees the combination of:

```text
shooter velocity + robot velocity

```

---

# 9. The Inverse Problem

Forward simulation asks:

> If I launch the Fuel with `(vx, vy, vz, ω)`, where does it land?

The inverse solver asks:

> What launch parameters will cause the Fuel to reach `(xt, yt, zt)`?

This is what `ProjectileSolver` does.

---

# 10. `ProjectileSolver`

The solver can potentially vary:

```text
v_mag
theta
phi
omega

```

depending on which parameters are fixed.

The method:

```python
get_active_parameters()

```

determines which variables are allowed to change.

For the trajectory surfaces, the important configuration is:

```python
fix_speed=True
fix_omega=True

```

Therefore the solver primarily adjusts the launch direction.

In other words:

```text
Shooter RPS
     ↓
 projectile speed + spin
     ↓
 FIXED during solve
     ↓
 solver adjusts launch direction

```

---

# 11. Levenberg-Marquardt

The inverse solver uses the **Levenberg-Marquardt algorithm**.

At a high level:

```text
Initial guess
     ↓
Simulate trajectory
     ↓
Measure target error
     ↓
Numerically calculate Jacobian
     ↓
Calculate parameter update
     ↓
Apply bounds
     ↓
Simulate again
     ↓
Accept/reject update
     ↓
Repeat

```

The residual consists of the difference between:

```text
simulated final state

```

and:

```text
desired target state

```

The project also allows a fourth residual component for Hub clearance.

---

# 12. Why the Jacobian Is Numerical

The projectile equations are nonlinear and the complete simulation contains numerical integration.

Instead of deriving an analytical Jacobian, the solver perturbs each active parameter by:

```python
eps = 1e-4

```

and observes the change in the residual.

Conceptually:

```text
∂residual / ∂parameter
    ≈
(residual(parameter + ε) - residual(parameter)) / ε

```

This is computationally expensive, which is one of the reasons the full solver is intended for **offline generation**, not continuous robot operation.

---

# 13. Solver Bounds

`ProjectileSolver` supports bounds for:

```text
speed
spin
theta
phi

```

The solver's `enforce_bounds()` function clips the current solution to those limits.

This is important because the inverse solver may mathematically find a solution that the physical robot cannot actually command.

For example:

```text
Mathematical solution:
hood angle = 83°

Robot:
hood can only reach 75°

→ solution is not mechanically usable

```

This distinction becomes extremely important later in the trajectory-surface pipeline.

---

# 14. Important `phi` / `theta` Naming Warning

**There is a genuine naming inconsistency in this repository.**

Different files use `phi` and `theta` differently.

The trajectory-surface pipeline documents the intended convention as:

```text
phi   = launch elevation / hood angle
theta = azimuth / turret angle

```

This is the convention used by:

```text
FitTrajectorySurface.py
GenerateValidShotProfile.py
PlotTrajectorySurfaces.py
valid_shots.csv

```

However, `ProjectilePath.py` contains a different mathematical parameterization:

```python
theta = arcsin(vz / v_mag)
phi   = arctan2(vy, vx)

```

and comments describing:

```text
theta = polar angle
phi   = azimuthal angle

```

Therefore:

> **Do not assume that** **`theta`** **and** **`phi`** **have the same definition everywhere in the repository.**

When working with the **trajectory surfaces and robot-facing models**, use:

```text
phi   → elevation
theta → azimuth

```

When modifying the underlying `ProjectileSolver`, read the actual velocity equations and parameterization rather than relying solely on the variable names.

This is one of the first things that should eventually be cleaned up in the codebase.

---

# 15. Another Angle-Parameterization Caveat

The solver currently reconstructs velocity using:

```python
vx = v_mag * cos(theta) * cos(phi)
vy = v_mag * cos(theta) * sin(phi)
vz = v_mag * sin(theta)

```

This parameterization is not the conventional:

```text
elevation + azimuth

```

spherical-coordinate definition that many users will expect.

In particular, the underlying solver's `theta` is derived through:

```python
arcsin(vz / v_mag)

```

which naturally corresponds to a range of approximately:

```text
-90° to +90°

```

Yet some surface code uses:

```text
theta = 0° → 340°

```

as the **azimuth**.

This is another reason to treat the surface/model naming as a higher-level convention and to be careful when modifying `ProjectilePath.py`.

---

# 16. Hub Clearance — `FuelClearance.py`

Hitting the Hub center is not the only requirement.

The projectile must also clear the physical edge of the Hub.

`FuelClearance.py` defines:

```python
hub_clearance(...)

```

which is passed into the inverse solver.

The current configuration uses approximately:

```text
Hub height        = 1.8288 m
Hub half-diagonal  = 0.5 m
Fuel diameter      = 0.1524 m
extra tolerance    = 0.2 m

```

The required clearance height is:

```text
Hub height
+ Fuel diameter
+ extra tolerance

```

The function walks backward through the simulated trajectory until it reaches the Hub edge, then determines whether the Fuel is high enough.

If it is too low, it returns a smooth penalty using:

```python
log(1 + exp(delta))

```

That penalty becomes part of the LM residual.

So the inverse solver is effectively trying to satisfy:

```text
hit target
+
clear Hub

```

rather than merely:

```text
hit target

```

---

# 17. Shooter Calibration

`TrajectorySurface.py` contains the current FRC 900 shooter conversion:

```python
speed_in_rps = abs((1 - 0.233333333) * rps)
speed = speed_in_rps * 0.31

spin_in_rps = abs(0.233333333 * rps)
spin = spin_in_rps * 2π

```

This means shooter RPS determines **both**:

* projectile launch speed
* projectile backspin

The constants are specific to the current shooter model/calibration.

They should not be treated as universal FRC constants.

If the robot changes:

* shooter wheel diameter
* wheel speed relationship
* compression
* shooter geometry
* wheel-to-ball interaction
* wheel configuration

then this conversion should be reevaluated.

---

# 18. Generating Trajectory Surfaces

`TrajectorySurfaces/TrajectorySurface.py` is where the expensive inverse-solving process is repeated over a large grid.

The independent variables are:

```text
r
vf
vl

```

where:

```text
r  = radial distance to Hub
vf = radial/forward robot velocity
vl = tangential/lateral robot velocity

```

The current grid is:

```text
r:
0 → 6.5 m
27 samples

vf:
-5.5 → +5.5 m/s
23 samples

vl:
-5.5 → +5.5 m/s
23 samples

```

The script skips states where:

```python
hypot(vf, vl) > 5.5

```

because the total horizontal robot speed is outside the modeled range.

---

# 19. What Happens at Every Grid Point?

For each:

```text
(r, vf, vl)

```

the script:

1. Places the target at:

   ```text
   xt = r
   yt = Hub height
   zt = 0

   ```
2. Computes an initial velocity guess aimed approximately at the target.
3. Creates a `ProjectileSolver`.
4. Fixes projectile speed.
5. Fixes projectile spin.
6. Allows the solver to modify the launch angles.
7. Runs Levenberg-Marquardt.
8. Simulates the resulting trajectory again.
9. Checks whether the final position is sufficiently close to the target.
10. Stores `theta` and `phi` if the shot is accepted.

---

# 20. Numerical Solution Validity vs. Mechanical Validity

This distinction is **extremely important**.

There are two different questions:

### Question 1: Does the physics solver find a trajectory?

This is a **numerical/physical validity** question.

The surface generator accepts a solution if its simulated endpoint is sufficiently close to the target.

Currently, the check is approximately:

```text
|x_final - x_target| < 0.5 m
|y_final - y_target| < 0.5 m
|z_final - z_target| < 0.5 m

```

### Question 2: Can the actual robot physically command that trajectory?

This is a **mechanical validity** question.

For example, suppose the inverse solver finds:

```text
elevation = 80°

```

but the robot's hood can only reach:

```text
45°–75°

```

The physics solution exists, but the robot cannot shoot it.

The later filtering stage therefore removes mathematically valid but mechanically impossible solutions.

---

# 21. Angle Bounds Are Robot-Specific

The angle limits in the repository:

```text
phi:   45°–75°
theta: 0°–340°
```

are **not universal properties of projectile motion**.

They represent the allowable angles for the particular shooter/turret design being modeled.

If a different robot has:

```text
hood range = 35°–70°
turret range = -170°–170°
```

then its valid bounds should be different.

This means the following values are **robot-design configuration**, not fundamental physics:

```python
PHI_MIN
PHI_MAX
THETA_MIN
THETA_MAX

```

They should be updated if:

* the hood mechanism changes
* the turret mechanism changes
* mechanical hard stops change
* wiring limits change
* software limits change
* a different robot uses the same ballistics code

The same principle applies to shooter calibration.

### Practical rule

Think of the project as having three categories of constants:

```text
Physics constants
    ↓
mass, radius, gravity, air density, drag, etc.

Game/field constants
    ↓
Hub height, Hub geometry, target location, etc.

Robot-specific constants
    ↓
shooter calibration, angle limits, mechanical ranges, etc.

```

Do not change a robot-specific parameter expecting the underlying projectile physics to change.

---

# 22. Saving the Raw Surface

The generated data is stored in:

```text
clean_theta_phi_surface.npz

```

It contains:

```text
r_vals
vf_vals
vl_vals
theta_surface
phi_surface

```

The surfaces have the conceptual form:

```text
theta_surface[r, vf, vl]
phi_surface[r, vf, vl]

```

A point can therefore be interpreted as:

```text
(r_vals[i], vf_vals[j], vl_vals[k])
          ↓
theta_surface[i,j,k]
phi_surface[i,j,k]

```

Invalid points are stored as:

```python
NaN

```

rather than being given fake angle values.

---

# 23. `FitTrajectorySurface.py`

This script turns the raw simulation data into fast polynomial models.

The fitting pipeline is:

```text
Raw surface
    ↓
Remove NaNs
    ↓
Remove mechanically invalid angles
    ↓
Remove statistical outliers
    ↓
Fit polynomial
    ↓
Export model

```

This filtering is important.

The polynomial should not be trained blindly on every numerical output produced by the inverse solver.

---

# 24. Angle Filtering

Before fitting, the script applies the configured mechanical bounds.

Currently:

```text
phi:
45° → 75°

theta:
0° → 340°

```

Any simulated point outside those ranges is discarded from the fitting dataset.

Again, these numbers are **specific to the modeled robot**.

If you change the mechanical design, update the angle limits before generating the final polynomial models.

---

# 25. Tukey IQR Outlier Filtering

After angle filtering, the fitting script performs **Tukey IQR outlier removal**.

This is an important part of the data-cleaning process.

For each radial-distance slice:

```text
r = constant

```

the script looks at the distribution of `phi` values.

It calculates:

```text
Q1 = 25th percentile
Q3 = 75th percentile

IQR = Q3 - Q1

```

and defines the Tukey fences:

```text
lower = Q1 - 1.5 × IQR
upper = Q3 + 1.5 × IQR

```

Any `phi` value outside those fences is removed from the fitting dataset.

This is the standard Tukey/IQR fence approach.

### Why do this?

The inverse solver can occasionally produce an unusual solution that is technically returned by the numerical optimizer but is inconsistent with the surrounding trajectory surface.

For example, neighboring states might look like:

```text
r = 4.0 m

phi:
52.1°
52.3°
52.5°
52.7°
52.6°
91.4°   ← anomalous
52.8°

```

A global polynomial fit can be distorted by such isolated points.

The Tukey filter removes these statistical anomalies before regression.

### Important limitation

Tukey filtering is a **statistical cleaning step**, not a physics-validity test.

An outlier is not automatically "wrong physics." It is simply unusual relative to the surrounding distribution.

The project uses it specifically as a practical way of cleaning the simulated surface before fitting.

---

# 26. Why Filtering Is Done Per `r`

The code does not compute one global IQR over the entire dataset.

Instead, it performs the Tukey calculation separately for each:

```text
r = constant

```

slice.

This matters because the expected elevation changes substantially with distance.

A value that looks unusual at:

```text
r = 1 m

```

might be completely normal at:

```text
r = 6 m

```

Using separate radial slices prevents the statistical distribution at one distance from determining the outlier threshold at another.

---

# 27. Polynomial Regression

After cleaning, the project uses:

```python
PolynomialFeatures(degree=4)
LinearRegression()

```

The result is a fourth-degree polynomial approximation.

The intended mappings in the surface pipeline are:

```text
phi   = f(r, vf, vl)
theta = f(r, vl)

```

In other words:

### Elevation

```text
(r, vf, vl)
      ↓
    phi

```

Elevation can depend on all three state variables.

### Azimuth

```text
(r, vl)
    ↓
  theta

```

Azimuth is modeled primarily as a function of distance and tangential velocity.

The radial velocity `vf` is intentionally omitted from the theta model.

This reflects the physical intuition that:

* radial velocity primarily changes the time/range problem
* tangential velocity primarily creates the lateral aiming correction

---

# 28. Why Not Just Use a Lookup Table?

The raw surface contains thousands of grid points.

A lookup table could work, but it has disadvantages:

* memory usage
* interpolation complexity
* discontinuities between grid cells
* additional lookup logic
* difficulty exporting directly to Java

A polynomial provides a compact approximation:

```text
(r, vf, vl)
       ↓
 polynomial
       ↓
 angle

```

The robot can evaluate this with a small number of arithmetic operations.

---

# 29. Polynomial Model Files

Each model is exported in two formats.

## `.pkl`

Example:

```text
phi_model.pkl
theta_model.pkl

```

These contain the complete scikit-learn model pipeline.

Use these for Python inference.

---

## `.json`

Example:

```text
phi_model.json
theta_model.json

```

These contain:

```json
{
    "intercept": ...,
    "coefs": [...],
    "powers": [...]
}

```

The polynomial is effectively:

```text
output =
    intercept
    + Σ coefficient_i
      × x^(power_i_x)
      × y^(power_i_y)
      ...

```

The JSON representation exists so that the model can be evaluated without installing Python or scikit-learn on the robot.

---

# 30. `PolynomialModel.java`

`LaunchJavaFiles/PolynomialModel.java` implements the JSON polynomial evaluator in Java.

It reads:

```text
intercept
coefs
powers

```

and evaluates the polynomial directly.

This is intended to be integrated into robot-side Java code.

One important detail:

> The Java evaluator itself does not know what its variables physically mean.

For example:

```text
phi model:
x = r
y = vf
...

```

while the theta model uses a different input set.

The caller is responsible for passing the correct variables in the correct order.

---

# 31. Valid Bounds

The fitted models should **not** be blindly evaluated for arbitrary inputs.

A fourth-degree polynomial will happily return a number for:

```text
r = 100 m
vf = 50 m/s
vl = -80 m/s

```

even though those values may be far outside the training data.

The polynomial's output may look reasonable while being completely meaningless.

`valid_bounds.json` is intended to prevent this kind of extrapolation.

The bounds include:

```text
global r range
global vf range
global vl range
angle bounds
per-r valid ranges

```

The per-`r` information is particularly useful because the valid velocity region can vary with distance.

---

# 32. Why Global Bounds Are Not Enough

Suppose the global model says:

```text
r ∈ [0, 6.5]
vf ∈ [-5.5, 5.5]
vl ∈ [-5.5, 5.5]

```

That does **not** imply every combination in that box is valid.

For example:

```text
r = 1 m
vf = +5.5 m/s
vl = +5.5 m/s

```

may not have a valid shot even though each individual value is inside the global range.

This is why `valid_bounds.json` also records **per-radial-distance bounds**.

The true feasible domain is generally a subset of the rectangular bounding box.

---

# 33. `GenerateValidShotProfile.py`

This script combines multiple shooter-speed surfaces.

Currently it considers:

```text
40 RPS
50 RPS
60 RPS

```

For each:

```text
(r, vf, vl)

```

it asks:

> Does this RPS produce a finite solution whose angles are mechanically achievable?

A solution is retained only if:

```text
phi is finite
theta is finite

AND

PHI_MIN <= phi <= PHI_MAX
THETA_MIN <= theta <= THETA_MAX

```

---

# 34. Multiple RPS Values Can Be Valid

A single robot state can have multiple valid shooter speeds.

For example:

```text
(r, vf, vl)
       ↓
 ┌─────┼─────┐
 ↓     ↓     ↓
40    50    60 RPS
 ✓     ✓     ✗

```

The output therefore intentionally contains multiple rows for the same:

```text
(r, vf, vl)

```

if multiple RPS settings work.

This allows a higher-level controller to make another decision, such as:

* prefer a particular RPS
* prefer a middle-range hood angle
* minimize shooter speed
* avoid mechanical limits
* choose the solution with the greatest robustness

`GenerateValidShotProfile.py` itself does **not** make that higher-level choice. It preserves all valid possibilities.

---

# 35. Valid Shot Table

The generated files are:

```text
TrajectorySurfaces/valid_shot_table/valid_shots.csv
TrajectorySurfaces/valid_shot_table/valid_shots.npz

```

The CSV columns are:

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

This table is therefore a convenient representation of:

```text
robot state
    +
shooter setting
    ↓
required angles

```

---

# 36. `LoadSurface.py` and Legacy CSV Files

There are two related data workflows in this repository.

The older workflow uses:

```text
valid_phi.csv
valid_theta.csv

```

and `LoadSurface.py` performs an inner join on:

```text
(r, vf, vl, rps)

```

to produce:

```text
valid_trajectories.csv

```

The newer workflow is:

```text
clean_theta_phi_surface.npz
        ↓
GenerateValidShotProfile.py
        ↓
valid_shots.csv
valid_shots.npz

```

Therefore, a new contributor should generally treat the `valid_shot_table` pipeline as the newer workflow.

`valid_trajectories.csv` and `valid_trajectories_mirrored.csv` are useful existing data artifacts, but they are not the central generation pipeline.

---

# 37. Mirrored Trajectory Data

The repository contains:

```text
valid_trajectories_mirrored.csv

```

This represents an additional processed trajectory dataset used by the verification script.

If working with this file, inspect its actual columns rather than assuming that it is identical to `valid_shots.csv`.

The important general rule is:

> Generated CSVs are data products from different stages of the pipeline, not interchangeable source-of-truth files.

---

# 38. `FuelPath.py` — Verification

`FuelPath.py` tests whether saved trajectory solutions can be reproduced by the full physics simulation.

The current verification configuration uses:

```python
N_SAMPLES = 20
RANDOM_SEED = 42

```

The script:

1. Loads saved valid trajectories.
2. Randomly selects 20.
3. Converts RPS to projectile speed and spin.
4. Reconstructs an inverse solver.
5. Runs Levenberg-Marquardt.
6. Simulates the resulting trajectory.
7. Measures the final error.
8. Reports angle errors and landing accuracy.

The fixed random seed means the same 20 samples are selected on repeated runs.

This makes it useful for regression testing after changing the physics model.

**Note:** `N_SAMPLES = 20` refers to the number of verification samples. It is **not a 20 RPS shooter profile**.

---

# 39. What `FuelPath.py` Actually Tests

It is useful to distinguish:

```text
surface generation

```

from:

```text
verification

```

A saved surface says:

> "The solver found this solution during generation."

`FuelPath.py` asks:

> "If I take this saved solution and run the solver/physics again, does it still reproduce the expected shot?"

This helps catch problems caused by:

* changed physics
* changed solver configuration
* changed calibration
* corrupted generated data
* inconsistent angle conventions
* differences between generation and verification

---

# 40. Plotting

`PlotTrajectorySurfaces.py` creates several diagnostic plots.

For each dataset:

```text
theta_surface.png
phi_surface.png
theta_overlay.png
phi_overlay.png
theta_fit.png
phi_fit.png

```

The **surface plots** show the raw numerical solver output.

The **fit plots** show the polynomial approximation.

The **overlay plots** allow you to visually compare:

```text
physics-derived surface
vs.
polynomial approximation

```

This is one of the best ways to identify a poor polynomial fit.

---

# 41. R² Is Not the Whole Story

The fitting script reports:

```text
R²

```

for each model.

A high R² is useful, but it should not be treated as proof that the model is safe for robot use.

A polynomial can have an excellent overall R² while:

* behaving poorly near a mechanical boundary
* oscillating near an edge of the training domain
* extrapolating badly
* having large local errors in a region that matters operationally

Therefore, inspect the overlay plots and, ideally, verify sampled predictions against the full physics solver.

---

# 42. Current RPS Configuration

The current generated trajectory datasets use:

```text
40 RPS
50 RPS
60 RPS

```

These are the RPS profiles used by the current valid-shot pipeline.

The corresponding directories are:

```text
40_RPS
50_RPS
60_RPS

```

and `GenerateValidShotProfile.py` uses:

```python
RPS_DIRS = {
    40: '40_RPS',
    50: '50_RPS',
    60: '60_RPS'
}

```

When adding or changing an RPS profile, make sure the generated directory, surface data, fitting configuration, and valid-shot generation configuration all agree.

---

# 43. RPS Configuration Across Scripts

RPS values are referenced by multiple parts of the pipeline, so they should remain synchronized.

In particular, check the RPS lists/dictionaries in:

```text
TrajectorySurface.py
FitTrajectorySurface.py
GenerateValidShotProfile.py
PlotTrajectorySurfaces.py
```

The current intended dataset configuration is:

```text
40 RPS
50 RPS
60 RPS
```

with directories:

```text
40_RPS
50_RPS
60_RPS
```

If you add a new shooter speed, update the relevant configuration and generate the corresponding surface/model data before expecting downstream scripts to find it.

---

# 44. Mechanical Bounds Must Stay Synchronized

The angle bounds appear in multiple scripts.

For example:

```text
FitTrajectorySurface.py
PlotTrajectorySurfaces.py
GenerateValidShotProfile.py
TrajectorySurface.py
FuelPath.py

```

may each contain their own copies.

This creates a significant maintenance risk.

If you change:

```text
hood range

```

or:

```text
turret range

```

you should search the entire repository for:

```text
PHI_MIN
PHI_MAX
THETA_MIN
THETA_MAX

```

and for literal angle values such as:

```text
45
75
340

```

Otherwise, different parts of the pipeline can disagree about what constitutes a valid shot.

---

# 45. Robot-Specific Bounds Should Be Treated as Configuration

A future cleanup would ideally define something like:

```python
ROBOT_CONFIG = {
    "phi_min": ...,
    "phi_max": ...,
    "theta_min": ...,
    "theta_max": ...,
    "shooter_calibration": ...,
}

```

and import it everywhere.

Currently, those values are duplicated.

Until that is refactored, assume that **changing the mechanical design requires changes in multiple files**.

---

# 46. Regenerating the Entire Pipeline

If you change only robot-side code, you probably do **not** need to regenerate anything.

If you change:

* projectile physics
* drag/lift model
* Fuel mass/radius
* Hub geometry
* shooter calibration
* shooter speed
* shooter spin
* robot velocity range
* distance range
* mechanical angle limits
* solver configuration

then the generated surfaces/models may no longer be valid.

The general regeneration sequence is:

```text
1. Update physics / robot configuration
              ↓
2. Generate trajectory surfaces
              ↓
3. Clean/filter surfaces
              ↓
4. Fit polynomial models
              ↓
5. Generate valid-shot table
              ↓
6. Plot/inspect models
              ↓
7. Run numerical verification

```

---

# 47. Surface Generation Is Expensive

The trajectory generator evaluates a large number of states.

The nominal grid contains:

```text
27 × 23 × 23

```

states:

```text
≈ 14,300 grid points

```

before skipping points outside the total robot-speed limit.

Each point can require multiple LM iterations, and each LM iteration requires multiple trajectory evaluations.

Therefore, regenerating a surface can be **very expensive**.

Do not run surface generation just because you want to evaluate an existing model.

---

# 48. When Should You Regenerate?

### Do NOT regenerate for:

* reading the models
* running Java evaluation
* plotting existing data
* testing an existing model
* changing unrelated robot code

### DO regenerate after changing:

* drag coefficient
* lift coefficient
* rotational drag
* Fuel mass/radius
* Hub height
* shooter calibration
* projectile speed
* projectile spin
* robot velocity range
* distance range
* mechanical angle limits
* solver behavior

---

# 49. A Useful Mental Model for Contributors

Think of the project as having four layers.

## Layer 1 — Physics

```text
ProjectilePath.py
FuelClearance.py

```

Answers:

> What does the Fuel physically do?

---

## Layer 2 — Numerical Ballistics

```text
ProjectileSolver
TrajectorySurface.py

```

Answers:

> What launch parameters make the Fuel reach the target?

---

## Layer 3 — Data Modeling

```text
FitTrajectorySurface.py
GenerateValidShotProfile.py

```

Answers:

> How can we turn thousands of expensive numerical solutions into something the robot can evaluate quickly?

---

## Layer 4 — Deployment

```text
PolynomialModel.java

```

Answers:

> How do we evaluate those models from robot code?

---

# 50. Recommended Reading Order

If you are completely new to the project, read the code in this order:

### 1. `ProjectilePath.py`

Understand:

* coordinate system
* RK4
* projectile physics
* velocity decomposition
* inverse solver
* LM algorithm

### 2. `FuelClearance.py`

Understand why simply reaching the target is not enough.

### 3. `TrajectorySurface.py`

Understand how thousands of inverse solutions are generated.

### 4. `FitTrajectorySurface.py`

This is especially important because it explains the data-cleaning process:

```text
NaN removal
→ mechanical filtering
→ Tukey IQR filtering
→ polynomial regression

```

### 5. `GenerateValidShotProfile.py`

Understand how multiple shooter speeds are combined into the valid-shot table.

### 6. `PlotTrajectorySurfaces.py`

Use the plots to understand what the resulting surfaces actually look like.

### 7. `FuelPath.py`

Understand how generated solutions are verified.

### 8. `PolynomialModel.java`

Read this when integrating the models into Java.

---

# 51. Common New-User Mistakes

### Mistake 1: Assuming `phi` and `theta` mean the same thing everywhere

They don't.

For the surface/model pipeline:

```text
phi   = elevation
theta = azimuth

```

But `ProjectilePath.py` uses a different underlying mathematical parameterization.

---

### Mistake 2: Treating 45°–75° as a physics limit

It isn't.

It is a **mechanical/robot-specific constraint**.

A different hood design may have completely different limits.

---

### Mistake 3: Changing the angle bound in one file

The limits are duplicated.

Search the whole repository.

---

### Mistake 4: Assuming every point inside the global bounds is valid

The feasible region is not necessarily a rectangular box.

Use the per-`r` bounds and actual valid-shot data.

---

### Mistake 5: Treating Tukey outliers as automatically physically impossible

The Tukey filter identifies statistically unusual points.

It does not prove that the underlying physics solution is wrong.

---

### Mistake 6: Using the polynomial outside its training range

The polynomial will still produce an answer.

That answer may be meaningless.

---

### Mistake 7: Assuming RPS only changes projectile speed

The current shooter model changes both:

```text
launch speed
spin

```

---

### Mistake 8: Regenerating surfaces unnecessarily

The inverse solver is expensive.

Use the existing `.npz` and model files whenever the underlying configuration has not changed.

---

### Mistake 9: Assuming every script uses the same RPS configuration automatically

The current pipeline uses:

```text
40 RPS
50 RPS
60 RPS
```

but RPS configuration is referenced by multiple scripts.

When changing the available shooter speeds, update all relevant configuration rather than assuming downstream scripts will automatically discover a new RPS directory.

---

### Mistake 10: Confusing robot velocity with shooter velocity

The solver's projectile velocity is combined with:

```text
vx_frame
vy_frame
vz_frame

```

to obtain the actual initial projectile velocity.

The robot's velocity is therefore part of the ballistics problem.

---

# 52. Quick Reference

| QuantityMeaning                |                                                 |
| ------------------------------ | ----------------------------------------------- |
| `r`                            | Horizontal/radial distance to Hub               |
| `vf`                           | Robot radial/forward velocity                   |
| `vl`                           | Robot tangential/lateral velocity               |
| `RPS`                          | Shooter rotational speed                        |
| `v_mag`                        | Projectile launch speed                         |
| `omega`                        | Projectile spin rate                            |
| `phi`                          | **Surface-pipeline elevation angle**            |
| `theta`                        | **Surface-pipeline azimuth angle**              |
| `vx, vy, vz`                   | Shooter-provided projectile velocity components |
| `vx_frame, vy_frame, vz_frame` | Robot/platform velocity components              |
| `dt`                           | RK4 timestep                                    |
| `PHI_MIN/MAX`                  | Robot-specific elevation limits                 |
| `THETA_MIN/MAX`                | Robot-specific azimuth limits                   |

---

# 53. The Entire Process in One Example

Suppose the robot reports:

```text
distance = 4.0 m
radial velocity = +1.5 m/s
tangential velocity = -0.8 m/s

```

and we are considering:

```text
50 RPS

```

The offline physics process has already determined a relationship like:

```text
(4.0, +1.5, -0.8)
        ↓
50 RPS
        ↓
required launch direction
        ↓
phi = elevation
theta = azimuth

```

Instead of running the full physics solver on the robot, the robot can evaluate the fitted polynomial:

```text
(r, vf, vl)
        ↓
phi model
        ↓
elevation

```

and:

```text
(r, vl)
        ↓
theta model
        ↓
azimuth

```

The robot then commands:

```text
hood → phi
turret → theta
shooter → 50 RPS

```

provided the state is inside the model's valid domain.

---

# 54. Final Takeaway

This project is fundamentally a process of converting a computationally expensive physics problem into a fast real-time approximation:

```text
             PHYSICS
                ↓
        Numerical trajectory
                ↓
       Inverse ballistics
                ↓
       Thousands of shots
                ↓
       Clean the dataset
          ↙           ↘
 angle limits       Tukey IQR
          ↘           ↙
          clean surface
                ↓
       Polynomial regression
                ↓
         JSON / Python
                ↓
             Java
                ↓
        Real-time shooting

```

The most important distinction to keep in mind is that **not every constraint in this project is a physics constraint**.

There are three different kinds of restrictions:

```text
Physics
  └─ gravity, drag, Magnus effect, etc.

Game/target
  └─ Hub height, Hub clearance, target geometry

Robot
  └─ hood range, turret range, shooter calibration,
     maximum robot velocity, usable RPS range

```

The polynomial models are only valid for the particular combination of those assumptions used to generate them.

Finally, the repository contains some historical differences between parts of the codebase—particularly the `phi`/`theta` naming, duplicated angle bounds, and multiple generations of CSV-based data products. The **current RPS datasets and valid-shot pipeline use 40, 50, and 60 RPS**. A new contributor should **follow the actual data flow and inspect the configuration at the bottom of each script before running it**, rather than assuming every configuration is automatically synchronized.

## AI Disclaimer

This README was written with assistance from OpenAI's ChatGPT and subsequently reviewed and edited by the project author for technical accuracy, clarity, and consistency with the implementation.
