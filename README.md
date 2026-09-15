# BallisticsMapper

A physics-based projectile simulation and inverse-ballistics pipeline
for a projectile launched from a moving platform.

The project models a projectile in three dimensions while accounting
for:

-   gravity
-   aerodynamic drag
-   the Magnus effect from spin
-   rotational drag torque

It then uses a fourth-order Runge--Kutta (RK4) numerical integrator to
simulate the forward trajectory and a Levenberg--Marquardt (LM) solver
to solve the inverse problem: given a target and platform motion,
determine launch angles that make the projectile reach the target.

For repeated real-time calculations, the expensive inverse solver can be
run offline over a grid of conditions. Those solutions are stored as
trajectory surfaces, filtered for mechanically valid shots, and
approximated with fourth-degree multivariable polynomial models. The
polynomial models are exported both as Python pickle files and as JSON
coefficient/power data that can be evaluated from Java.

------------------------------------------------------------------------

## 1. Pipeline

The repository is organized around the following pipeline:

``` text
Physics Model
     |
     v
RK4 Numerical Approximation
     |
     v
Levenberg–Marquardt Inverse Solver
     |
     v
Trajectory Surfaces
     |
     v
Valid-Shot Selection
     |
     v
Polynomial Models
     |
     v
Java / Robot-Side Evaluation
```

### What each stage does

1.  **Physics Model**\
    `ProjectilePath.py` defines the projectile dynamics and the
    equations for acceleration and spin decay.

2.  **RK4 Approximation**\
    The nonlinear ordinary differential equations are numerically
    integrated with a fourth-order Runge--Kutta method.

3.  **Levenberg--Marquardt Inverse Solver**\
    The solver varies the launch-angle parameters until the simulated
    projectile reaches the requested target, subject to configured
    bounds and optional clearance constraints.

4.  **Trajectory Surfaces**\
    `TrajectorySurfaces/TrajectorySurface.py` repeatedly runs the
    inverse solver over a grid of radial distance, radial velocity, and
    tangential velocity. The resulting launch angles form
    multidimensional surfaces.

5.  **Valid-Shot Selection**\
    `TrajectorySurfaces/GenerateValidShotProfile.py` examines several
    shooter-speed/RPS datasets and keeps every state for which both
    launch angles fall inside the configured mechanical limits.

6.  **Polynomial Models**\
    `TrajectorySurfaces/FitTrajectorySurface.py` fits fourth-degree
    polynomial regressions to the valid numerical solutions. The
    resulting models provide a much faster approximation of the inverse
    solver.

7.  **Robot-Side Evaluation**\
    `LaunchJavaFiles/PolynomialModel.java` shows how the exported JSON
    representation can be loaded and evaluated in Java.

------------------------------------------------------------------------

# 2. Repository Structure

The important source and generated files are:

``` text
BallisticsMapper/
├── README.md
├── .gitignore
├── BallisticsMapper.pdf
│
├── ProjectilePath.py
├── FuelClearance.py
├── FuelPath.py
│
├── RK4ExampleOutputs/
│   └── R=0.1_M=0.3_Vx0=8_Vy0=9_Vz0=-4_omega0=90_dt=0.01/
│       ├── xpos.png
│       ├── ypos.png
│       ├── zpos.png
│       ├── xvel.png
│       ├── yvel.png
│       ├── zvel.png
│       └── pos.png
│
├── LaunchJavaFiles/
│   └── PolynomialModel.java
│
└── TrajectorySurfaces/
    ├── TrajectorySurface.py
    ├── FitTrajectorySurface.py
    ├── GenerateValidShotProfile.py
    ├── LoadSurface.py
    ├── PlotTrajectorySurfaces.py
    │
    ├── valid_phi.csv
    ├── valid_theta.csv
    ├── valid_trajectories.csv
    ├── valid_trajectories_mirrored.csv
    │
    ├── valid_shot_table/
    │   ├── valid_shots.csv
    │   └── valid_shots.npz
    │
    └── <RPS dataset>/
        ├── clean_theta_phi_surface.npz
        ├── models/
        │   ├── phi_model.pkl
        │   ├── phi_model.json
        │   ├── theta_model.pkl
        │   ├── theta_model.json
        │   └── valid_bounds.json
        └── plots/
            ├── phi_fit.png
            ├── phi_overlay.png
            ├── phi_surface.png
            ├── theta_fit.png
            ├── theta_overlay.png
            └── theta_surface.png
```

The ZIP also contains a `.venv` directory and macOS metadata files. `BallisticsMapper.pdf` is the project report, and `RK4ExampleOutputs/` contains example RK4 simulation plots. The
`.venv` is a local Python environment and should not be committed to a
shared/forked repository. A fresh virtual environment should be created
instead.

------------------------------------------------------------------------

# 3. Core Physics: `ProjectilePath.py`

`ProjectilePath.py` contains the main simulation and inverse-solver
implementation.

It has two primary classes:

-   `RungeKutta4`
-   `Projectile`
-   `ProjectileSolver`

## `RungeKutta4`

This is a general fourth-order Runge--Kutta integrator.

It stores:

-   current time
-   current state vector
-   timestep
-   derivative function
-   time history
-   state history

The `rk4()` method performs one RK4 step. The `sim()` method repeatedly
advances the state until the requested simulation time is reached.

The integrator can also stop early when the simulated projectile falls
below a specified target height.

## `Projectile`

`Projectile` defines the physical model.

A projectile is initialized with its radius and mass:

``` python
fuel = pp.Projectile(0.0762, 0.226796)
```

The default physical parameters in the code include:

-   radius: `0.0762 m`
-   mass: `0.226796 kg`
-   air density: `1.195 kg/m^3`
-   dynamic viscosity: `1.835e-5 N·s/m^2`
-   gravitational acceleration: `9.81 m/s^2`
-   drag coefficient: `0.47`
-   rotational drag coefficient: `0.02`
-   lift coefficient: `1.5 * S`, where `S` is the spin ratio

The moment of inertia defaults to:

``` text
I = 0.4 m R^2
```

### Coordinate convention

The simulation uses Cartesian components:

``` text
x — radial/horizontal direction toward the target
y — vertical direction
z — lateral/tangential direction
```

The velocity is represented as a vector:

``` text
v = (vx, vy, vz)
```

Gravity acts in the negative `y` direction.

Only spin about the `z` axis is modeled. The spin direction is
represented by `n_hat`, where positive corresponds to backspin and
negative corresponds to topspin.

### Important methods

-   `reynolds(speed)`\
    Computes Reynolds number.

-   `omega(t, speed)`\
    Computes angular velocity after rotational drag.

-   `shear(t, speed)`\
    Computes the spin ratio used by the lift model.

-   `dvxdt(...)`, `dvydt(...)`, `dvzdt(...)`\
    Compute the Cartesian acceleration components.

-   `dvdt(...)`\
    Returns the complete acceleration vector.

-   `trajectory(...)`\
    Simulates velocity and position over time using RK4.

-   `plot_solutions(...)`\
    Produces velocity, position, and 3D trajectory plots.

## `ProjectileSolver`

`ProjectileSolver` solves the inverse problem.

You provide:

-   target position `(xt, yt, zt)`
-   an initial velocity guess
-   initial angular velocity
-   platform/frame velocity
-   optional bounds
-   optional fixed speed and spin settings
-   simulation settings
-   an optional clearance function

The solver represents the launch direction using the project's angle
convention:

-   `theta` = elevation angle
-   `phi` = azimuthal angle

The important distinction is that `vx`, `vy`, and `vz` are scalar
Cartesian velocity components, while `(vx, vy, vz)` together form the
velocity vector.

### Levenberg--Marquardt

The LM solver repeatedly:

1.  simulates the projectile
2.  calculates the target residual
3.  numerically estimates the Jacobian
4.  forms a damped normal-equation system
5.  updates the active launch parameters
6.  accepts or rejects the step based on the resulting residual

The implementation can keep launch speed and angular velocity fixed
while solving only for the launch angles. This is the configuration used
when generating the trajectory surfaces.

------------------------------------------------------------------------

# 4. Clearance Model: `FuelClearance.py`

`FuelClearance.py` contains the Hub-clearance function used by the
inverse solver.

``` python
hub_clearance(...)
```

The function checks the trajectory near the Hub and applies a smooth
penalty when the projectile is below the required clearance height at
the Hub edge.

The configured values include:

-   Hub target height: `1.8288 m`
-   Hub half-diagonal: `0.5 m`
-   projectile diameter: `0.1524 m`
-   additional tolerance: `0.2 m`

The required clearance height is calculated from these values.

This function is passed into `ProjectileSolver` as `clearance_func`.

If you adapt this project to a different target, this is one of the
files you should modify.

------------------------------------------------------------------------

# 5. Random Numerical Verification: `FuelPath.py`

`FuelPath.py` is a verification script.

It:

1.  loads `valid_trajectories_mirrored.csv`
2.  randomly selects a fixed number of entries
3.  converts shooter RPS into projectile speed and spin
4.  reconstructs a `ProjectileSolver` for each selected state
5.  runs the LM solver
6.  simulates the resulting trajectory
7.  compares the resulting landing point with the target
8.  prints errors and a final accuracy summary

The script currently uses:

``` python
N_SAMPLES = 20
RANDOM_SEED = 42
```

The fixed seed makes the random sample repeatable.

This script is useful after changing the physics model or generated data
because it tests whether saved inverse solutions can be reproduced by
the full simulator.

------------------------------------------------------------------------

# 6. Generating Trajectory Surfaces

## `TrajectorySurfaces/TrajectorySurface.py`

This is the most computationally expensive stage.

`gen_surface()` evaluates the inverse solver across a grid of:

-   `r` --- radial distance to target
-   `vf` --- radial/forward platform velocity
-   `vl` --- tangential/lateral platform velocity

The current grid is:

``` text
r  : 0 to 6.5 m, 27 samples
vf : -5.5 to +5.5 m/s, 23 samples
vl : -5.5 to +5.5 m/s, 23 samples
```

Points where:

``` python
hypot(vf, vl) > 5.5
```

are skipped.

For each remaining state, the LM solver determines the launch angles
required to hit the target.

A solution is retained only if the final simulated position is within
`0.5 m` of the target in each Cartesian coordinate.

The output is:

``` text
clean_theta_phi_surface.npz
```

containing:

``` text
r_vals
vf_vals
vl_vals
theta_surface
phi_surface
```

### Shooter RPS

The script contains a conversion from shooter RPS to projectile speed
and spin based on the current shooter model:

``` python
speed_in_rps = abs((1 - 0.233333333) * rps)
speed = speed_in_rps * 0.31

spin_in_rps = abs(0.233333333 * rps)
spin = spin_in_rps * 2 * pi
```

If your shooter geometry, wheel ratio, or wheel diameter changes, update
this function.

### Running surface generation

From the directory containing `BallisticsMapper/`:

``` bash
python BallisticsMapper/TrajectorySurfaces/TrajectorySurface.py
```

The script's currently active entry point calls:

``` python
rps_20()
```

so a fresh run of the file generates the `20_RPS_T` dataset.

Other RPS functions are present in the script but commented out. To
generate other speeds, uncomment or add the desired calls/functions.

The existing repository also contains precomputed datasets for several
RPS values. These are data artifacts, not regenerated automatically by
every script.

### Runtime warning

This process runs thousands of inverse simulations and can take a long
time. Do not run it merely to use the existing polynomial models. Only
regenerate surfaces when you intentionally change the physics, target
geometry, operating range, solver configuration, or shooter model.

------------------------------------------------------------------------

# 7. Surface Fitting: `FitTrajectorySurface.py`

`FitTrajectorySurface.py` converts the numerical trajectory surfaces
into polynomial surrogate models.

The fitting process is:

``` text
NPZ surface
    |
    +--> remove NaNs
    |
    +--> enforce angle limits
    |
    +--> remove outliers
    |
    +--> polynomial regression
    |
    +--> export model
```

## Data filtering

The configured mechanical angle ranges are:

``` text
45° <= theta <= 75°     elevation
0°  <= phi   <= 340°    azimuth
```

The script removes:

-   non-finite surface values
-   values outside the configured angle ranges

It then performs a Tukey IQR outlier filter per radial-distance slice on
the elevation data used by the current implementation.

## Polynomial models

The models are fourth-degree polynomial regressions.

The intended mathematical mappings are:

``` text
theta = f(r, vf, vl)
phi   = g(r, vl)
```

The azimuth model does not use radial velocity because the simulated
data indicated that the azimuth is effectively independent of radial
velocity over the modeled range.

The implementation uses scikit-learn:

``` python
PolynomialFeatures(4)
LinearRegression()
```

and reports the training-set `R²` for each fitted model.

## Outputs

For each processed RPS dataset, the script writes:

``` text
models/phi_model.pkl
models/phi_model.json
models/theta_model.pkl
models/theta_model.json
models/valid_bounds.json
```

### `.pkl`

The pickle files contain the complete scikit-learn pipeline and are
convenient for Python inference.

### `.json`

The JSON files contain:

``` text
intercept
coefs
powers
```

This representation is intentionally simple so that the polynomial can
be evaluated outside Python.

For example, each term is effectively:

``` text
coefficient * x^power_x * y^power_y
```

summed with the intercept.

### `valid_bounds.json`

This contains the valid global input ranges, angle limits, and
per-radial-distance bounds generated from the cleaned fitting data. It
can be used to avoid extrapolating the polynomial model outside its
modeled domain.

### Running the fitter

From the directory containing `BallisticsMapper/`:

``` bash
python BallisticsMapper/TrajectorySurfaces/FitTrajectorySurface.py
```

The checked-in generated model directories are:

``` text
40_RPS
50_RPS
60_RPS
```

The fitting script's `dir_list` is still configured with older directory names, so update it to match the current datasets before running it.

If you create a different RPS dataset, add its directory to `dir_list`.

------------------------------------------------------------------------

# 8. Valid-Shot Selection: `GenerateValidShotProfile.py`

This script combines multiple RPS-specific trajectory surfaces.

Currently it uses:

``` python
RPS_DIRS = {
    40: '40_RPS',
    50: '50_RPS',
    60: '60_RPS'
}
```

For each `(r, vf, vl)` state, it checks whether each RPS has:

-   a finite elevation result
-   a finite azimuth result
-   an elevation inside the allowed range
-   an azimuth inside the allowed range

Every RPS that passes is retained.

This means the output can contain multiple valid shooter-speed choices
for the same `(r, vf, vl)` state.

The script also prints coverage statistics, including:

-   total grid points
-   points with at least one valid RPS
-   points with exactly one valid RPS
-   points with exactly two valid RPS values
-   points with all three RPS values

Outputs:

``` text
TrajectorySurfaces/valid_shot_table/valid_shots.csv
TrajectorySurfaces/valid_shot_table/valid_shots.npz
```

The CSV contains:

``` text
r
vf
vl
rps
phi_rad
theta_rad
phi_deg
theta_deg
```

This stage is a **feasibility/selection stage**, not another numerical
optimization.

### Running it

From the directory containing `BallisticsMapper/`:

``` bash
python BallisticsMapper/TrajectorySurfaces/GenerateValidShotProfile.py
```

The three configured RPS surface directories must exist.

------------------------------------------------------------------------

# 9. Combining Existing CSV Data: `LoadSurface.py`

`LoadSurface.py` is an older/simple data-combination utility.

It reads:

``` text
valid_phi.csv
valid_theta.csv
```

and performs an inner join using:

``` text
(r, vf, vl, rps)
```

It then writes:

``` text
valid_trajectories.csv
```

This is useful when the angle data are stored in separate CSV files and
you want a combined table containing both angles.

Because the repository also contains the newer
`valid_shot_table/valid_shots.csv` workflow, treat `LoadSurface.py` as a
data utility rather than the primary modern pipeline.

------------------------------------------------------------------------

# 10. Plotting: `PlotTrajectorySurfaces.py`

`PlotTrajectorySurfaces.py` loads the saved NPZ surfaces and fitted
pickle models and generates visual comparisons.

For each configured RPS directory, it produces:

### `theta_surface.png`

The raw numerical elevation surface for the zero-radial-velocity slice.

### `phi_surface.png`

The raw numerical azimuth surface for the zero-tangential-velocity
slice.

### `theta_overlay.png`

The numerical elevation surface overlaid with the polynomial fit.

### `phi_overlay.png`

The numerical azimuth surface overlaid with the polynomial fit.

### `theta_fit.png`

The polynomial elevation model by itself.

### `phi_fit.png`

The polynomial azimuth model by itself.

Run from the directory containing `BallisticsMapper/`:

``` bash
python BallisticsMapper/TrajectorySurfaces/PlotTrajectorySurfaces.py
```

The checked-in generated model directories are:

``` text
40_RPS
50_RPS
60_RPS
```

The plotting script's `dir_list` is still configured with older directory names, so update it to match the current datasets before running it.

------------------------------------------------------------------------

# 11. Java Integration: `LaunchJavaFiles/PolynomialModel.java`

`PolynomialModel.java` is a lightweight Java evaluator for the JSON
polynomial representation.

It uses Jackson to deserialize:

``` json
{
  "intercept": ...,
  "coefs": [...],
  "powers": [...]
}
```

The `evaluate(x, y)` method calculates:

``` text
intercept + sum(
    coefficient_i
    * x^(power_i_x)
    * y^(power_i_y)
)
```

This allows the Python-generated polynomial to be evaluated on the robot
side without installing Python, NumPy, or scikit-learn.

## Important

The Java file is only the polynomial evaluator. It does not:

-   generate trajectory surfaces
-   run the physics simulation
-   run the LM solver
-   choose an RPS
-   perform input validation automatically

The surrounding robot code must provide the correct input variables and
select the desired model.

The Java file imports Jackson:

``` java
import com.fasterxml.jackson.databind.ObjectMapper;
```

so a Java project using this class needs Jackson available on its
classpath.

------------------------------------------------------------------------

# 12. Generated Data and Model Files

The repository contains both source code and precomputed outputs.

## `clean_theta_phi_surface.npz`

A NumPy archive containing the raw inverse-solver surfaces.

Variables:

``` text
r_vals
vf_vals
vl_vals
theta_surface
phi_surface
```

Use these files when you want to inspect or refit the numerical
solutions without rerunning the expensive LM simulations.

## `models/*.pkl`

Python/scikit-learn versions of the fitted polynomial models.

## `models/*.json`

Language-independent polynomial representation intended for deployment.

## `models/valid_bounds.json`

Input and angle-domain metadata for safe inference.

## `plots/*.png`

Diagnostic plots generated from the surfaces and models.

## `valid_phi.csv`

Stored valid azimuth/elevation data in CSV form.

## `valid_theta.csv`

Stored valid elevation data in CSV form.

## `valid_trajectories.csv`

Combined valid trajectory table created by `LoadSurface.py`.

## `valid_trajectories_mirrored.csv`

An expanded/mirrored valid trajectory table included in the project for
verification and downstream use.

## `valid_shot_table/valid_shots.csv`

Current multi-RPS valid-shot table.

## `valid_shot_table/valid_shots.npz`

NumPy version of the same valid-shot table, including angle bounds and
RPS metadata.

------------------------------------------------------------------------

# 13. Installing and Running From a Fork

The recommended workflow is to fork or clone the repository and create a
new Python environment rather than using the `.venv` included in the
ZIP.

## 13.1 Fork

On GitHub, fork the repository into your own account.

Then clone your fork:

``` bash
git clone <your-fork-url>
cd BallisticsMapper
```

Or, if you are starting from the ZIP:

``` bash
unzip BallisticsMapper.zip
cd BallisticsMapper
```

If the ZIP contains the original `.venv`, it is recommended to delete it
before creating your own environment.

------------------------------------------------------------------------

## 13.2 Create a virtual environment

Python 3.10+ is recommended.

macOS/Linux:

``` bash
python3 -m venv .venv
source .venv/bin/activate
```

Windows PowerShell:

``` powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
```

Windows Command Prompt:

``` cmd
python -m venv .venv
.venv\Scripts\activate
```

------------------------------------------------------------------------

## 13.3 Install Python dependencies

The scripts use packages including:

``` text
numpy
matplotlib
scikit-learn
pandas
tabulate
```

Install them with:

``` bash
python -m pip install numpy matplotlib scikit-learn pandas tabulate
```

If you modify the project, keep the environment reproducible by
recording the installed packages:

``` bash
python -m pip freeze > requirements.txt
```

Then another user can recreate the environment with:

``` bash
python -m pip install -r requirements.txt
```

A `requirements.txt` file is not currently included in the project, so
the command above is also a convenient first step when preparing a fork
for public distribution.

------------------------------------------------------------------------

# 14. Running the Existing Models

If you only want to use the existing precomputed data and models, **do
not regenerate the trajectory surfaces**.

From the directory containing `BallisticsMapper/`, use:

``` bash
python BallisticsMapper/TrajectorySurfaces/PlotTrajectorySurfaces.py
```

to inspect the surfaces and polynomial fits, and:

``` bash
python BallisticsMapper/FuelPath.py
```

to run the numerical verification workflow.

Be aware that some scripts use repository-relative paths. Run them from
the repository root:

``` text
BallisticsMapper/
```

rather than changing into `TrajectorySurfaces/`.

------------------------------------------------------------------------

# 15. Rebuilding Everything

If you want to reproduce the pipeline from scratch after changing the
physics or robot parameters, use this general order:

## Step 1 --- Modify the physics

Edit:

``` text
ProjectilePath.py
```

Change projectile properties, aerodynamic coefficients, gravity, spin
model, or other physical assumptions as required.

If the target clearance geometry changes, also modify:

``` text
FuelClearance.py
```

## Step 2 --- Generate trajectory surfaces

Edit the RPS configuration in:

``` text
TrajectorySurfaces/TrajectorySurface.py
```

Then run:

``` bash
python TrajectorySurfaces/TrajectorySurface.py
```

This produces the numerical inverse solutions.

## Step 3 --- Fit polynomial models

Update the RPS directory list in:

``` text
TrajectorySurfaces/FitTrajectorySurface.py
```

Then run:

``` bash
python TrajectorySurfaces/FitTrajectorySurface.py
```

This generates the `.pkl`, `.json`, and `valid_bounds.json` model files.

## Step 4 --- Generate the valid-shot table

Update `RPS_DIRS` in:

``` text
TrajectorySurfaces/GenerateValidShotProfile.py
```

Then run:

``` bash
python TrajectorySurfaces/GenerateValidShotProfile.py
```

## Step 5 --- Inspect the fits

Run from the directory containing `BallisticsMapper/`:

``` bash
python BallisticsMapper/TrajectorySurfaces/PlotTrajectorySurfaces.py
```

## Step 6 --- Verify selected trajectories

Run:

``` bash
python FuelPath.py
```

This lets you compare saved solutions against fresh numerical
simulations.

------------------------------------------------------------------------

# 16. Adapting the Project to Another Robot or Game

This project is not limited to its original robot configuration, but
several constants are currently specific to the existing setup.

At minimum, review:

### Projectile parameters

In `ProjectilePath.py`:

``` text
radius
mass
moment of inertia
air density
dynamic viscosity
gravity
drag coefficient
lift coefficient
rotational drag coefficient
```

### Target geometry

In `FuelClearance.py`:

``` text
hub height
hub geometry
projectile diameter
clearance tolerance
target center
```

### Shooter conversion

In:

``` text
TrajectorySurfaces/TrajectorySurface.py
FuelPath.py
```

review:

``` text
get_frc900_spin_and_speed_from_shooter_rps(...)
```

This converts shooter RPS into projectile launch speed and spin.

### Robot velocity convention

The trajectory-surface generator uses:

``` text
vf = radial/forward platform velocity
vl = tangential/lateral platform velocity
```

and maps those quantities into the projectile solver's Cartesian frame.

If your robot coordinate system is different, change the conversion
consistently everywhere.

### Angle limits

The current mechanical ranges are:

``` text
elevation theta: 45°–75°
azimuth phi:     0°–340°
```

These limits appear in the surface-fitting, valid-shot, and plotting
scripts. If the mechanism changes, update them consistently.

------------------------------------------------------------------------

# 17. Coordinate and Angle Conventions

For consistency when modifying the project:

## Cartesian velocity

``` text
v = (vx, vy, vz)
```

where:

``` text
vx = x component
vy = y component
vz = z component
```

The velocity vector is:

``` text
v⃗ = [vx, vy, vz]
```

and its magnitude is:

``` text
|v⃗| = sqrt(vx² + vy² + vz²)
```

## Platform motion

The surface-generation code interprets:

``` text
vf = radial velocity
vl = tangential velocity
```

The radial velocity changes the effective forward launch velocity, while
tangential velocity changes the launch direction relative to the target.

## Launch angles

This project's paper/code convention is:

``` text
theta = elevation angle
phi   = azimuthal angle
```

When modifying scripts, preserve this convention. Some older variable
names/comments in the fitting and plotting scripts use the opposite
legacy names, so take care when changing those files.

------------------------------------------------------------------------

# 18. Common Problems

## `FileNotFoundError`

Several scripts use paths beginning with:

``` text
BallisticsMapper/...
```

Run the scripts from the directory containing the `BallisticsMapper`
folder if you are using the outer project layout, or update the paths if
you have made the repository itself the working directory.

The safest approach is to inspect the path strings in the script you are
running and execute it from the directory those paths expect.

If you reorganize the repository, converting these scripts to paths
based on `__file__` is recommended.

## Missing RPS directory

For example:

``` text
40_RPS
50_RPS
60_RPS
```

must exist before fitting or generating the multi-RPS valid-shot table.

## Missing model files

`PlotTrajectorySurfaces.py` expects the `.pkl` files produced by
`FitTrajectorySurface.py`.

Run the fitting script first if you intentionally regenerated a surface.

## Polynomial gives strange results

Do not assume a polynomial is valid outside its training domain.

Use:

``` text
valid_bounds.json
```

to determine the modeled input range.

The polynomial is a surrogate approximation of the numerical physics
solver, not a replacement for the underlying physics outside the
generated domain.

## Surface generation is extremely slow

This is expected. Surface generation calls the numerical simulator and
LM inverse solver many times.

If you only need launch-angle inference, use the already-generated
polynomial models.

------------------------------------------------------------------------

# 19. Recommended Git Setup

The repository should not normally commit:

``` text
.venv/
__pycache__/
*.pyc
.DS_Store
```

The included `.gitignore` is currently empty, so a fork should add these
entries.

A reasonable starting `.gitignore` is:

``` gitignore
# Python
.venv/
venv/
__pycache__/
*.py[cod]
*$py.class

# macOS
.DS_Store

# IDEs
.vscode/
.idea/

# Python tooling
.pytest_cache/
.mypy_cache/

# Optional local outputs
*.log
```

Generated trajectory surfaces, models, CSV files, and plots are
different: they are useful project artifacts and may be intentionally
committed if you want a fork to contain ready-to-use models.

------------------------------------------------------------------------

# 20. Suggested Development Workflow

When changing the project, isolate changes by stage.

### Physics change

``` text
ProjectilePath.py
       ↓
TrajectorySurface.py
       ↓
FitTrajectorySurface.py
       ↓
GenerateValidShotProfile.py
       ↓
Plot / Verify
```

### Mechanical-angle change

Usually update:

``` text
FitTrajectorySurface.py
GenerateValidShotProfile.py
PlotTrajectorySurfaces.py
TrajectorySurface.py
```

### Shooter conversion change

Update the RPS-to-speed/spin conversion consistently in every script
that uses it.

### Polynomial deployment change

If the mathematical model stays the same but the robot-side
implementation changes, the JSON models can generally remain unchanged.

------------------------------------------------------------------------

# 21. What Should Be Regenerated After a Change?

  --------------------------------------------------------------------------
  Change                    Regenerate Refit polynomials? Rebuild valid-shot
                             surfaces?                                table?
  ----------------- ------------------ ------------------ ------------------
  Projectile                       Yes                Yes                Yes
  mass/radius                                             

  Drag/lift model                  Yes                Yes                Yes

  Spin model                       Yes                Yes                Yes

  Target                           Yes                Yes                Yes
  height/geometry                                         

  Shooter                          Yes                Yes                Yes
  RPS-to-speed                                            
  conversion                                              

  Solver bounds                    Yes                Yes                Yes

  Mechanical angle     No, if surfaces                Yes                Yes
  limits only            already exist                    

  Plot formatting                   No                 No                 No
  only                                                    

  Java evaluator                    No                 No                 No
  only                                                    

  Robot-side                        No                 No                 No
  polynomial                                              
  selection logic                                         
  --------------------------------------------------------------------------

The general rule is: if the numerical solution changes, regenerate the
surfaces. If the valid domain or model-fitting procedure changes, refit
the polynomial models.

------------------------------------------------------------------------

# 22. Data Flow Summary

A complete run can be thought of as:

``` text
ProjectilePath.py
    |
    | forward physics simulation
    v
TrajectorySurface.py
    |
    | inverse solve over (r, vf, vl)
    v
clean_theta_phi_surface.npz
    |
    +-----------------------------+
    |                             |
    v                             v
FitTrajectorySurface.py    GenerateValidShotProfile.py
    |                             |
    v                             v
Polynomial JSON/PKL         valid_shots.csv / NPZ
    |
    v
PolynomialModel.java
    |
    v
Fast robot-side angle evaluation
```

The expensive physics and inverse solver therefore run offline, while
the final polynomial evaluation can be performed very quickly during
operation.

------------------------------------------------------------------------

# 23. Important Implementation Notes

### The polynomial is a surrogate, not machine learning from experimental data

The polynomial models are fitted to solutions generated by the physics
simulation. They are not trained on a dataset of experimentally measured
shots.

The numerical solver is the source of the modeled launch-angle data;
polynomial regression is used to approximate those solutions
efficiently.

### Multiple valid shooter speeds can exist

The valid-shot table intentionally preserves every RPS value that
satisfies the angle constraints for a given `(r, vf, vl)` state.

A later robot-control layer can choose among those options according to
whatever strategy is desired.

### Avoid extrapolation

The polynomial model should be used within the domain represented by the
generated surface. `valid_bounds.json` exists partly to support
inference-time domain checking.

### Keep notation consistent

In particular:

``` text
theta = elevation
phi   = azimuth
```

and:

``` text
(vx, vy, vz) = velocity vector components
```

This distinction matters when translating the Python implementation into
another language.

------------------------------------------------------------------------

# 24. Minimal Quick Start

If you just want to inspect the existing project:

``` bash
git clone <your-fork-url>
cd BallisticsMapper

python3 -m venv .venv
source .venv/bin/activate

python -m pip install numpy matplotlib scikit-learn pandas tabulate

cd ..
python BallisticsMapper/TrajectorySurfaces/PlotTrajectorySurfaces.py
```

If you want to verify saved trajectories:

``` bash
python BallisticsMapper/FuelPath.py
```

If you intentionally want to regenerate the inverse-solver surfaces:

``` bash
python BallisticsMapper/TrajectorySurfaces/TrajectorySurface.py
```

then fit them:

``` bash
python BallisticsMapper/TrajectorySurfaces/FitTrajectorySurface.py
```

then rebuild the valid-shot table:

``` bash
python BallisticsMapper/TrajectorySurfaces/GenerateValidShotProfile.py
```

and finally inspect the results:

``` bash
python BallisticsMapper/TrajectorySurfaces/PlotTrajectorySurfaces.py
```

------------------------------------------------------------------------

# 25. Project Philosophy

The central idea of this project is to separate **physical accuracy**
from **runtime computational cost**.

The full physics simulation and inverse solver provide the detailed
solution:

``` text
target state
    ↓
physics simulation + numerical inverse solve
    ↓
required launch angles
```

That process is too expensive to repeat continuously on a real-time
robot controller.

Instead, the project performs the expensive computation offline:

``` text
many simulated states
    ↓
trajectory surfaces
    ↓
valid solutions
    ↓
polynomial approximation
```

The robot can then evaluate a compact polynomial:

``` text
(r, vf, vl)
    ↓
theta, phi
```

with the goal of obtaining the benefits of the detailed physics model
without repeatedly running the full numerical inverse solver during
operation.

ChatGPT was used in the making of this README file.
