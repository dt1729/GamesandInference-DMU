using iLQGames:
    GeneralGame, ProximityCost, Unicycle4D, NPlayerUnicycleCost,
    generate_nplayer_navigation_game, n_states, n_controls, xyindex, xindex,
    SystemTrajectory, iLQSolver, uindex, transform_to_feedbacklin, cost, plot_traj,
    plot_traj!, horizon, dynamics, player_costs, proximitycost, solve!, n_players,
    ravoid, trajectory!, samplingtime, initialtime, time_disc2cont, time_cont2disc, LinearSystem, AffineStrategy
import iLQGames: dx
using StaticArrays
using Random
using POMDPs: POMDPs
using LinearAlgebra
using Infiltrator


####################################################################
# Setup game variables for multiple agents - Done
# Initialise game instance - Done
# Solve for control actions(iLQSolver) - Done 
####################################################################
function bicycle_dynamics_solver(prev_state, controls, delta_t)
    new_state = [0.0,0.0,0.,0.]
    l = 3
    new_state[1] = prev_state[1] + prev_state[4]*cos(prev_state[3])
    new_state[2] = prev_state[2] + prev_state[4]*sin(prev_state[3])
    new_state[3] = prev_state[3] + controls[1]
    new_state[4] = prev_state[4] + controls[2]
    return new_state
end


# generate a game
T_horizon = 10.
ΔT = 0.1
dt = 0.1
"--------------------------------- Unicycle4D ---------------------------------"

x01 = @SVector [-3., 0., 0., 0.]
x02 = @SVector [0.,  3., -pi/2, 0.]
x03 = @SVector [-3.,  3., -pi/4, 0.]
x04 = @SVector [3.0, 0, -pi, 0]
x05 = @SVector [10.0, 0, 0, 0]
x0 = vcat(x01, x02, x03, x04, x05)
# goal states (goal position of other player with opposite orientation)
xg1 = @SVector [3., 0., 0., 0.]
xg2 = @SVector [0., -3., -pi/2, 0.]
xg3 = @SVector [3., -3., -pi/4, 0.]
xg4 = @SVector [-3., 0., 0, 0.]
xg5 = @SVector [-10.0, 0, -pi, 0]
g = generate_nplayer_navigation_game(Unicycle4D, NPlayerUnicycleCost, T_horizon,
                                     ΔT, xg1, xg2, xg3, xg4,xg5;
                                     proximitycost=ProximityCost([2.0, 2.0, 2.0, 2.0, 2.0],
                                                                 [0., 50.0, 50.0, 50.0, 50.0]))
dyn = dynamics(g)
nx = n_states(dyn)
nu = n_controls(dyn)
pcs = player_costs(g)
h = horizon(g)
zero_op = zero(SystemTrajectory{h, ΔT, nx, nu})

# quad_sanity_check(g)

# solve the lq game
solver = iLQSolver(g; state_regularization=50.0, control_regularization=20.0)
# - setup initial_strategy
steer_init(k::Int) = cos(k/h*pi) * deg2rad(0)
acc_init(k::Int) = -cos(k/h*pi)*0.1
γ_init = SizedVector{h}([AffineStrategy((@SMatrix zeros(nu, nx)),
(@SVector [steer_init(k), 0.7*acc_init(k),
            steer_init(k), acc_init(k),
            steer_init(k), acc_init(k),
            steer_init(k), acc_init(k),
            steer_init(k), acc_init(k)])) for k in 1:h])

# loop starts here
for i = 0:dt:10

    global x0, x01, x02, x03, x04, x05, nu, nx, k, h, zero_op, g, solver, γ_init, dyn, pcs, zero_op

    # generate initial operating point from simulating initial strategy
    # solve the game
    a,b,c = solve!(copy(zero_op), copy(γ_init), g, solver, SVector{20}(x0))  #converged, current_op, current_strategy
    zero_op = b 
    # Call bicycle_dynamics_solver for 5 agents 
    x01 = bicycle_dynamics_solver(x01, SVector{2}(b.u[1][1:2]), dt)
    x02 = bicycle_dynamics_solver(x02, SVector{2}(b.u[1][3:4]), dt)
    x03 = bicycle_dynamics_solver(x03, SVector{2}(b.u[1][5:6]), dt)
    x04 = bicycle_dynamics_solver(x04, SVector{2}(b.u[1][7:8]), dt)
    x05 = bicycle_dynamics_solver(x05, SVector{2}(b.u[1][9:10]), dt)
    # store data 
    x0 = vcat(x01, x02, x03, x04, x05)
    # update initial conditions(states and controls as this setup)
end


@infiltrate
