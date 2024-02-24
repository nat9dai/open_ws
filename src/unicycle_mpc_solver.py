import casadi as cs
import opengen as og

T  = 10  # The MPC horizon length
T_sep = 7
N  = 3   # Maximum number of agents in the network
NX = 6   # The number of elements in the state vector
NY = 4   # The number of elements in the output vector
NU = 2   # The number of elements in the control vector
NP = 2   # The number of elements in the position vector eg. 2 for ground vehicle, 3 for UAV
NO = 50 # The number of the obstacles
sampling_time = 0.1
d_sep = 0.5

# discrete-time unicycle dynamics
def dynamics_dt(x, u):
    return cs.vcat([x[0] + sampling_time * u[0] * cs.cos(x[2]),
                    x[1] + sampling_time * u[0] * cs.sin(x[2]),
                    x[2] + sampling_time * u[1],
                    u[0] * cs.cos(x[2]),
                    u[0] * cs.sin(x[2]),
                    u[1]])

# The stage cost for x and u
def stage_cost(_x, _u, _y_bar, _Q, _R, _gamma):
    y = cs.vcat([_x[0], _x[1], _x[3], _x[4]]) # x, y, v_x, v_y
    dy = y - _y_bar
    return _gamma * cs.mtimes([dy.T, _Q, dy]) + cs.mtimes([_u.T, _R, _u])

def panalty_sep(_x, _p, _gamma, _N_t):
    p_i = cs.vcat([_x[0], _x[1]])
    sum_d = 0
    for i in range(N-1):
        dp = p_i - _p[i]
        sum_d += _N_t[i] * cs.mtimes([dp.T, dp])
    return _gamma * sum_d

def separation_constraints(_x, _p, _N_t):
    f1 = []
    fmax = [cs.inf] * T_sep * (N-1)
    fmin = [d_sep] * T_sep * (N-1)
    for i in range(T_sep):
        p_i = cs.vcat([_x[i][0], _x[i][1]])
        for j in range(N-1):
            index = i * (N-1) + j
            dp = p_i - _p[index]
            f1 = cs.vertcat(f1, cs.mtimes([dp.T, dp]))
    C1 = og.constraints.Rectangle(fmin, fmax)
    return [f1, C1]

def obstacle_constraints(_x, _obs, _r_min):
    f2 = []
    for i in range(T):
        p_i = cs.vcat([_x[i][0], _x[i][1]])
        for obstacle in _obs:
            r = obstacle - p_i
            f2 = cs.vertcat(f2, cs.fmax(0, _r_min**2 - cs.mtimes([r.T, r])))
    return f2

# Input (optimization variables)
u_k = [cs.MX.sym('u_' + str(i), NU) for i in range(T)]

# Parameters
x_t = [cs.MX.sym('x_' + str(i), NX) for i in range(T)]
y_bar_t = [cs.MX.sym('y_bar_' + str(i), NY) for i in range(T)]
q_t = cs.MX.sym('q_0') # Q
r_t = cs.MX.sym('r_0', NU) # R
N_t = cs.MX.sym('N_0', N-1) # The binary array eg. [1,1,0] means there are 2 detected neighboring agents
gamma_t = [cs.MX.sym('gamma_' + str(i)) for i in range(T)] # Discount factors (gamma)
rho_t = cs.MX.sym('rho_0') # rho_sep
r_min_t = cs.MX.sym('r_min_0') # minimum distance from obstacles
p_t = [cs.MX.sym('p_' + str(i) + '_' + str(j), NP) for i in range(T) for j in range(N-1)]
# Obstacle
obs_i = [cs.MX.sym('obs_' + str(i), NP) for i in range(NO)]

# Cohesion/alignment dynamic trade-off implement on Q matrix
Q = cs.MX(NY, NY)
Q[0, 0] = 1-q_t
Q[1, 1] = 1-q_t
Q[2, 2] = q_t
Q[3, 3] = q_t

# Input weights R
R = cs.MX(NU, NU)
for i in range(NU):
    R[i, i] = r_t[i]

# Create the cost function

total_cost = 0

for t in range(0, T-1):
    total_cost += stage_cost(x_t[t], u_k[t], y_bar_t[t], Q, R, gamma_t[t])  # update cost
    if t > T_sep:
        # Calculate the start and end indices for the slice of p_t that corresponds to time step t
        start_index = t * (N-1)
        end_index = start_index + (N-1)
        total_cost += rho_t * panalty_sep(x_t[t], p_t[start_index:end_index], gamma_t[t], N_t)
    x_t[t+1] = dynamics_dt(x_t[t], u_k[t])  # update state

total_cost += cs.mtimes([u_k[-1].T, R, u_k[-1]])

umin = [-2.0, -10.0] * T  # - cs.DM.ones(NU * N) * cs.inf
umax = [2.0, 10.0] * T  # cs.DM.ones(NU * N) * cs.inf

input_bounds = og.constraints.Rectangle(umin, umax)

f1, C1 = separation_constraints(x_t, p_t, N_t)
f2 = obstacle_constraints(x_t, obs_i, r_min_t)

optimization_variables = []
optimization_parameters = []

optimization_variables  += u_k

optimization_parameters += [x_t[0]]
optimization_parameters += y_bar_t
optimization_parameters += [q_t]
optimization_parameters += [r_t]
optimization_parameters += [N_t]
optimization_parameters += gamma_t
optimization_parameters += [rho_t]
optimization_parameters += [r_min_t]
optimization_parameters += p_t
optimization_parameters += obs_i

optimization_variables = cs.vertcat(*optimization_variables)
optimization_parameters = cs.vertcat(*optimization_parameters)

problem = og.builder.Problem(optimization_variables,
                             optimization_parameters,
                             total_cost) \
    .with_aug_lagrangian_constraints(f1, C1) \
    .with_penalty_constraints(f2) \
    .with_constraints(input_bounds)

ros_config = og.config.RosConfiguration() \
    .with_package_name("open_nmpc_controller") \
    .with_node_name("open_mpc_controller_node") \
    .with_rate((int)(1.0/sampling_time)) \
    .with_description("Cool ROS node.")

build_config = og.config.BuildConfiguration()\
    .with_build_directory("optimization_engine")\
    .with_build_mode("release")\
    .with_build_c_bindings()

meta = og.config.OptimizerMeta()\
    .with_optimizer_name("unicycle_mpc_controller")

solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-5)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)
builder.build()
