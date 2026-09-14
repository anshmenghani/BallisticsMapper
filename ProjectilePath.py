"""
This script employs a Runge-Kutta 4 antiderivative approximation of the 
nonlinear ordinary differential equations that govern the motion of an
object through a fluid. These differential equations include the change 
in x, y, and z velocities of the object, along with the change in x, y, 
and z positions of the object.

The simulation accounts for the following:
1. Gravity
2. Drag
3. Magnus Force (lift)
4. Rotational drag torque

The simulation assumes the following:
- Gravity acts along the -y direction.
- Only spin about the z-axis is present (i.e., topspin or backspin).

Suppose we are given a target set of coordinates, but want to know the 
speed, theta, phi, and spin (or the x, y, and z velocities) and spin 
needed to move the object to those coordinates. This is essentially the 
inverse of what the RK4 simulation is doing. Since RK4 is a numerical 
approximator in itself, we cannot analytically derive a closed-form inverse. 

Thus, we employ the Levenberg-Marquardt algorithm (a combination of the
Newton-Raphson method and gradient descent) to change the velocity, launch 
angle, and spin inputs until we reach a solution. The program is designed 
to allow us to limit the solution velocity and spins based on physical
and angular constraints. 
"""

import numpy as np 
import matplotlib.pyplot as plt


# Runge-Kutta 4 numerical integrator 
class RungeKutta4:
    def __init__(self, dadt, t0, dt, a0):
        # t and a are initialized to the differential equation solution's initial conditions
        
        self.t = t0 # time
        self.a = a0 # rk4 approximation
        self.dt = dt # timestep
        self.dadt = dadt # differential equation

        self.t_list = [self.t] # list of times
        self.a_list = [self.a] # list of RK4-approximations

    # approximate the differential equation's antiderivative
    def rk4(self):
        k1 = self.dt*self.dadt(self.t, self.a)
        k2 = self.dt*self.dadt(self.t+self.dt/2., self.a+k1/2.)
        k3 = self.dt*self.dadt(self.t+self.dt/2., self.a+k2/2.)
        k4 = self.dt*self.dadt(self.t+self.dt, self.a+k3)
        
        self.a = self.a + ((k1+2*k2+2*k3+k4)/6.)
        self.t += self.dt

    # run until the simulation reaches the designated end time 
    def sim(self, timelen=10, stop_on_y=None):
        while self.t <= timelen: # stop RK4 when it reaches specified simulation time
            if stop_on_y is not None: # stop RK4 when y-position < target y position
                if self.a[1] < stop_on_y and np.sign(self.dadt(self.t, self.a)[1]) == -1: # make sure projectile is falling before stopping 
                    break 

            self.rk4()
            self.t_list.append(self.t)
            self.a_list.append(self.a.copy())


class Projectile:
    def __init__(self, radius, mass, I=False, rho=1.195, mu=1.835e-5, g=9.81, spin_coeff=lambda Re: 0.02, drag_coeff=lambda Re: 0.47, lift_coeff=lambda S: 1.5 * S):
        self.radius = radius # object radius, m 
        self.csarea = np.pi * (self.radius**2) # object cross-sectional area, m^2
        self.mass = mass # object mass, kg
        if I: # object moment of inertia, kg*m^2
            self.I = I
        else:
            self.I = 0.4 * self.mass * (self.radius**2) 
        
        self.rho = rho # fluid density, kg/m^3
        self.mu = mu # fluid dynamic viscosity, N*s/m^2
        self.g = g # acceleration due to gravity, m/s^2

        self.spin_coeff = spin_coeff # rotational drag coefficient  
        self.drag_coeff = drag_coeff # coefficient of drag
        self.lift_coeff = lift_coeff # coefficient of lift (Magnus effect)

        self.omega0 = 0 # object initial angular velocity, rad/s
        self.n_hat = 1 # angular velocity direction unit vector (+1: backspin, -1: topspin, 0: no spin)

    # calculate Reynold's number
    def reynolds(self, speed):
        return (self.rho*speed*2*self.radius) / self.mu

    # angular velocity as a function of time
    def omega(self, t, speed):
        if self.omega0 < 1e-8:
            return 0.0
        denom1 = 1 / self.omega0
        denom2 = (t*self.spin_coeff(self.reynolds(speed))*self.rho*(self.radius**5)) / (2*self.I) 
        return self.n_hat / (denom1+denom2)

    # calculate shear parameter of the object 
    def shear(self, t, speed):
        if speed < 1e-8:
            return 0.0
        return (np.abs(self.omega(t, speed))*self.radius) / speed

    # change in x-velocity vs. time 
    def dvxdt(self, t, speed, vx, vy):
        coeff = (-1*self.rho*self.csarea*speed) / (2*self.mass) # coefficient of constants in the general dv_x/dt formula 
        w = self.omega(t, speed)
        return coeff * ((self.drag_coeff(self.reynolds(speed))*vx) + (self.lift_coeff(self.shear(t, speed))*np.sign(w)*vy))

    # change in y-velocity vs. time
    def dvydt(self, t, speed, vx, vy):
        coeff = (self.rho*self.csarea*speed) / (2*self.mass) # coefficient of constants in the general dv_y/dt formula 
        w = self.omega(t, speed)
        return coeff * ((-1*self.drag_coeff(self.reynolds(speed))*vy) + (self.lift_coeff(self.shear(t, speed))*np.sign(w)*vx)) - self.g 

    # change in z-velocity vs. time
    def dvzdt(self, speed, vz):
        coeff = (-1*self.rho) / (2*self.mass) # coefficient of constants in the general dv_z/dt formula 
        return coeff * self.drag_coeff(self.reynolds(speed)) * self.csarea * speed * vz

    # change in velocity vector vs. time
    def dvdt(self, t, a):
        vx, vy, vz = a
        speed = np.linalg.norm(a)

        dvx = self.dvxdt(t, speed, vx, vy)
        dvy = self.dvydt(t, speed, vx, vy)
        dvz = self.dvzdt(speed, vz)

        return np.array([dvx, dvy, dvz], dtype=float)

    # function to simulate velocity and position over time 
    def trajectory(self, vx0, vy0, vz0, omega0=0, n_hat=1, sx0=0, sy0=0, sz0=0, t0=0, dt=0.01, sim_end_time=10, stop_on_y=None):
        # set initial conditions 
        self.omega0 = omega0 # initial angular velocity (rad/s)
        self.n_hat = n_hat # initial angular velocity direction

        v0_vec = np.array([vx0, vy0, vz0], dtype=float) # vector of initial velocities (m/s)
        p0_vec = np.array([sx0, sy0, sz0], dtype=float) # vector of initial positions (m)

        # approximate velocity of object
        vel_approx = RungeKutta4(self.dvdt, t0, dt, v0_vec)
        vel_approx.sim(timelen=sim_end_time)
        v_list = vel_approx.a_list
        vx_list, vy_list, vz_list = map(list, zip(*vel_approx.a_list))

        # change in position vector vs. time
        def dsdt(t, _):
            idx = int(t / dt)
            dsx = v_list[idx][0] 
            dsy = v_list[idx][1]
            dsz = v_list[idx][2]

            return np.array([dsx, dsy, dsz], dtype=float)

        # approximate position of object
        pos_approx = RungeKutta4(dsdt, t0, dt, p0_vec)
        pos_approx.sim(timelen=sim_end_time, stop_on_y=stop_on_y)
        sx_list, sy_list, sz_list = map(list, zip(*pos_approx.a_list))

        speedf = np.linalg.norm(v_list[(len(sx_list)-1)]) # final speed
        omegaf = self.omega(vel_approx.t_list[(len(sx_list)-1)], speedf) # final angular velocity

        return vel_approx.t_list, vx_list, vy_list, vz_list, speedf, pos_approx.t_list, sx_list, sy_list, sz_list, omegaf

    # plot the outputs of the simulation
    def plot_solutions(self, vel_approx_tlist, vx_list, vy_list, vz_list, pos_approx_tlist, sx_list, sy_list, sz_list):
        plt.scatter(vel_approx_tlist[:len(pos_approx_tlist)], vx_list[:len(pos_approx_tlist)], s=1, color="red")
        plt.xlabel("Time (s)")
        plt.ylabel("X-Velocity (m/s)")
        plt.title("X-Velocity vs. Time")
        plt.show()

        plt.scatter(vel_approx_tlist[:len(pos_approx_tlist)], vy_list[:len(pos_approx_tlist)], s=1, color="blue")
        plt.xlabel("Time (s)")
        plt.ylabel("Y-Velocity (m/s)")
        plt.title("Y-Velocity vs. Time")
        plt.show()

        plt.scatter(vel_approx_tlist[:len(pos_approx_tlist)], vz_list[:len(pos_approx_tlist)], s=1, color="green")
        plt.xlabel("Time (s)")
        plt.ylabel("Z-Velocity (m/s)")
        plt.title("Z-Velocity vs. Time")
        plt.show()

        plt.scatter(pos_approx_tlist, sx_list, s=1, color="red")
        plt.xlabel("Time (s)")
        plt.ylabel("X-Position (m)")
        plt.title("X-Position vs. Time")
        plt.show()

        plt.scatter(pos_approx_tlist, sy_list, s=1, color="blue")
        plt.xlabel("Time (s)")
        plt.ylabel("Y-Position (m)")
        plt.title("Y-Position vs. Time")
        plt.show()

        plt.scatter(pos_approx_tlist, sz_list, s=1, color="green")
        plt.xlabel("Time (s)")
        plt.ylabel("Z-Position (m)")
        plt.title("Z-Position vs. Time")
        plt.show()

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        t_f = pos_approx_tlist
        scf = ax.scatter(
            sx_list, sz_list, sy_list,
            c=t_f, s=6, cmap='cividis'
        )
        plt.colorbar(scf, ax=ax, label='Time--Full Model (s)')
        ax.plot(sx_list, sz_list, sy_list, linewidth=1, label="Full Trajectory", color="olive")
        
        xmin, xmax = np.min(sx_list), np.max(sx_list)
        ymin, ymax = np.min(sy_list), np.max(sy_list)
        zmin, zmax = np.min(sz_list), np.max(sz_list)

        overall_min = min(xmin, ymin, zmin, 0) 
        overall_max = max(xmax, ymax, zmax)

        ax.set_xlim(overall_min, overall_max)
        ax.set_ylim(overall_min, overall_max)
        ax.set_zlim(overall_min, overall_max)

        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Z Position (m)')
        ax.set_zlabel('Y Position (m)')
        ax.set_title('3D Trajectory with Time')
        ax.legend()

        plt.show()


# class to solve for  active parameters (v_vec, theta, phi, and omega0 [v_vec and omega0 can be held constat if needed]) given a target x, y, z 
class ProjectileSolver:
    def __init__(self, projectile, xt, yt, zt, vx0, vy0, vz0, omega0, vx_frame=0, vy_frame=0, vz_frame=0, omega_frame=0, v_bounds=(-np.inf, np.inf), fix_speed=False, omega_bounds=(-np.inf, np.inf), theta_bounds=(-np.inf, np.inf), fix_omega=False, phi_bounds=(-np.inf, np.inf), ct=0, clearance_func=lambda *args: 0, x_scale=0.05, y_scale=0.05, z_scale=0.05, c_scale=0.2, n_hat=1, sx0=0, sy0=0, sz0=0, t0=0, dt=0.01, sim_end_time=10, eps=1e-4, lam0=1e-2, tol=False, lm_iters=20, lam_scaleup=8, lam_scaledown=0.2):
        self.projectile = projectile # projectile object being solved
        self.targets = np.array([xt, yt, zt, ct]) # target x (m), y (m), z (m), clearance (defined if object needs to clear a physical threshold, clearance != 0 if physical threshold is not cleared)
        self.scale_array = np.array([x_scale, y_scale, z_scale, c_scale]) # weight of each parameter on residual calculation  

        self.sx0 = sx0 # initial x-position (m)
        self.sy0 = sy0 # initial y-position (m) 
        self.sz0 = sz0 # initial z-position (m)

        self.vx = vx0 # x-velocity (vx0=initial guess) (m/s)
        self.vy = vy0 # y-velocity (vy0=initial guess) (m/s)
        self.vz = vz0 # z-velocity (vz0=initial guess) (m/s)
        self.vx_frame = vx_frame # initial x-velocity (m/s)
        self.vy_frame = vy_frame # initial y-velocity (m/s)
        self.vz_frame = vz_frame # initial z-velocity (m/s) 
        self.v_min, self.v_max = v_bounds # defines range of possible velocity magnitudes
        self.v = np.array([self.vx, self.vy, self.vz]) # velocity vector (m/s), not used in any function
        self.v_mag = np.linalg.norm(self.v) # velocity magnitude (m/s)
        self.fix_speed = fix_speed # keep speed magnitude constant; should program change speed

        self.omega = omega0 # angular velocity (omega0=initial guess) (rad/s)
        self.omega_min, self.omega_max = omega_bounds # defines range of possible angular velocities 
        self.omega_frame = omega_frame # initial spin (rad/s)
        self.n_hat = n_hat # angular velocity direction unit vector (+1: backspin, -1: topspin, 0: no spin)
        self.fix_omega = fix_omega # keep spin magnitude constant; should program change spin

        self.theta = np.arcsin(self.vz / self.v_mag) # theta (rad, polar angle down from the +z axis)
        self.phi = np.arctan2(self.vy, self.vx) # phi (rad, azimuthal angle in the x–y plane, measured from the +x-axis)
        self.theta_min, self.theta_max = theta_bounds # defines range of possible thetas
        self.phi_min, self.phi_max = phi_bounds # defines range of possible phis

        self.clearance_func = clearance_func # function associated with current clearance 

        self.t0 = t0 # initial time (s)
        self.dt = dt # timestep (s)
        self.sim_end_time = sim_end_time # simulation duration (s)
        
        self.eps = eps # Jacobian approximation delta t (s)
        self.lam = lam0 # Levenberg–Marquardt damping parameter

        if tol: # solution accuracy tolerance 
            self.tol = tol
        else:
            self.tol = np.full(self.targets.shape[0], 1e-4) 

        self.lm_iters = lm_iters # max Levenberg–Marquardt iterations 
        self.lam_scaleup = lam_scaleup # lambda scaleup factor
        self.lam_scaledown = lam_scaledown # lambda scaledown factor

        self._shot_cache = {} # RK4 run cache

    # finds which parameters are we changing 
    def get_active_parameters(self):
        params = []

        if not self.fix_speed:
            params.append("v_mag")

        params.append("theta")
        params.append("phi")

        if not self.fix_omega:
            params.append("omega")

        return params

    def update_velocity_from_angles(self):
        self.vx = self.v_mag * np.cos(self.theta) * np.cos(self.phi)
        self.vy = self.v_mag * np.cos(self.theta) * np.sin(self.phi)
        self.vz = self.v_mag * np.sin(self.theta)
    
    def enforce_bounds(self):
        self.omega = np.clip(self.omega, self.omega_min, self.omega_max) # clip angular velocity

        v = np.array([self.vx, self.vy, self.vz])
        v_mag = np.linalg.norm(v)
        v = np.clip(v_mag, self.v_min, self.v_max) # clip speed

        theta = np.arcsin(self.vz / v) 
        theta = np.clip(theta, self.theta_min, self.theta_max) # clip theta
        phi = np.arctan2(self.vy, self.vx)
        phi = np.clip(phi, self.phi_min, self.phi_max) # clip phi

        # reconstruct vx, vy, and vz from new theta and phi
        self.update_velocity_from_angles()
        self.v = np.array([self.vx, self.vy, self.vz])
        self.theta = theta
        self.phi = phi
    
    # simulate shot and return final state 
    def rk4_shot(self):
        # account for initial velocities 
        vx_total = self.vx + self.vx_frame
        vy_total = self.vy + self.vy_frame
        vz_total = self.vz + self.vz_frame
        omega_total = self.omega + self.omega_frame

        key = (vx_total, vy_total, vz_total, omega_total)
        if key in self._shot_cache: # check if we've already run this RK4 simulation
            return self._shot_cache[key]

        vel_approx_tlist, vx_list, vy_list, vz_list, speedf, pos_approx_tlist, sx_list, sy_list, sz_list, omegaf = self.projectile.trajectory(vx_total, vy_total, vz_total, omega0=omega_total, n_hat=self.n_hat, sx0=self.sx0, sy0=self.sy0, sz0=self.sz0, t0=self.t0, dt=self.dt, sim_end_time=self.sim_end_time, stop_on_y=self.targets[1])
        xf, yf, zf = sx_list[-1], sy_list[-1], sz_list[-1]
        cf = self.clearance_func(vel_approx_tlist, vx_list, vy_list, vz_list, speedf, pos_approx_tlist, sx_list, sy_list, sz_list, omegaf)

        out = np.array([xf, yf, zf, cf])
        self._shot_cache[key] = out  # update RK4 cache dictionary
        return out
    
    # difference between current final state and desired final state
    def residual(self):
        self.update_velocity_from_angles()
        shot_output = self.rk4_shot()
        return (shot_output-self.targets) / self.scale_array
    
    # matrix of change in residual with respect to change in active parameters 
    def jacobian(self):
        r0 = self.residual()
        with_respect_to = self.get_active_parameters()

        J = np.zeros((r0.shape[0], len(with_respect_to)))

        for i, name in enumerate(with_respect_to):
            setattr(self, name, getattr(self, name)+self.eps)

            if name in ["v_mag", "theta", "phi"]:
                self.update_velocity_from_angles()

            r = self.residual()
            J[:, i] = (r-r0) / self.eps

            setattr(self, name, getattr(self, name)-self.eps)

            if name in ["v_mag", "theta", "phi"]:
                self.update_velocity_from_angles()

        return J

    # Levenberg–Marquardt algorithm: combination of Newton-Raphson method and gradient descent to minimize the cost function of the residual 
    def levenberg_marquardt(self):
        for _ in range(self.lm_iters):
            self._shot_cache.clear()

            # calculate residual of current active parameters
            r = self.residual()
            cost = np.dot(r, r)

            J = self.jacobian()
            H = J.T @ J # Hessian shortcut
            
            diag_H = np.diag(np.diag(H))
            A = H + self.lam * (np.eye(H.shape[0])+diag_H)
            B = -(J.T @ r)

            delta = np.linalg.solve(A, B) # attempt to solve Ax = B, where x is the change in active parameters

            vxi, vyi, vzi, omegai = self.vx, self.vy, self.vz, self.omega # save original inputs

            # update active parameters
            params = self.get_active_parameters()

            for i, name in enumerate(params):
                setattr(self, name, getattr(self, name) + delta[i])

            if any(p in ["v_mag", "theta", "phi"] for p in params):
                self.update_velocity_from_angles()            
            self.enforce_bounds()

            # calculate new residual 
            r_new = self.residual()
            cost_new = np.dot(r_new, r_new)

            if cost_new < cost: # if change lowers cost
                self._shot_cache.clear() # clear RK4 run cache
                self.lam *= self.lam_scaledown # scale down lambda, move closer to Newton-Raphson (faster)
                if np.all(np.abs(self.residual()) < self.tol): # check if a solution has been reached
                    return True
            else:
                self.vx, self.vy, self.vz, self.omega = vxi, vyi, vzi, omegai # revert back to original inputs
                self.lam *= self.lam_scaleup # scale up lambda, move closer to gradient descent (more stable)

        return False

ball = Projectile(0.1, 0.3)
vel_approx_t_list, vx_list, vy_list, vz_list, speedf, pos_approx_t_list, sx_list, sy_list, sz_list, omegaf = ball.trajectory(8, 9, -4, 90, stop_on_y=0)
ball.plot_solutions(vel_approx_t_list, vx_list, vy_list, vz_list, pos_approx_t_list, sx_list, sy_list, sz_list)
