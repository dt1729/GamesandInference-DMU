using iLQGames:
    GeneralGame, ProximityCost, Unicycle4D, NPlayerUnicycleCost,
    generate_nplayer_navigation_game, n_states, n_controls, xyindex, xindex,
    SystemTrajectory, iLQSolver, uindex, transform_to_feedbacklin, cost, plot_traj,
    plot_traj!, horizon, dynamics, player_costs, proximitycost, solve, n_players,
    ravoid, trajectory!, samplingtime, initialtime, time_disc2cont, time_cont2disc,
    @S
import iLQGames: dx

using StaticArrays
using Random
using POMDPs: POMDPs
using LinearAlgebra
using Infiltrator


####################################################################
# Setup game variables for multiple agents - Done
# Chain the game(David Kiel Cpp impl)
# Initialise game instance
# Solve for control actions(iLQSolver)
####################################################################

struct Unicycle <: ControlSystem{ΔT,nx,nu} end
# state: (px, py, phi, v)
dx(cs::Unicycle, x, u, t) = SVector(x[4]cos(x[3]), x[4]sin(x[3]), u[1], u[2])

dynamics_1 = Unicycle()
dynamics_2 = Unicycle()
dynamics_3 = Unicycle()
dynamics_4 = Unicycle()
dynamics_5 = Unicycle()

cost_1 = (FunctionPlayerCost((g, x, u, t) -> (x[1]^2 + x[2]^2 + u[1]^2)),
         FunctionPlayerCost((g, x, u, t) -> ((x[4] - 1)^2 + u[2]^2)))
cost_2 = (FunctionPlayerCost((g, x, u, t) -> (x[1]^2 + x[2]^2 + u[1]^2)),
         FunctionPlayerCost((g, x, u, t) -> ((x[4] - 1)^2 + u[2]^2)))
cost_3 = (FunctionPlayerCost((g, x, u, t) -> (x[1]^2 + x[2]^2 + u[1]^2)),
         FunctionPlayerCost((g, x, u, t) -> ((x[4] - 1)^2 + u[2]^2)))
cost_4 = (FunctionPlayerCost((g, x, u, t) -> (x[1]^2 + x[2]^2 + u[1]^2)),
         FunctionPlayerCost((g, x, u, t) -> ((x[4] - 1)^2 + u[2]^2)))
cost_5 = (FunctionPlayerCost((g, x, u, t) -> (x[1]^2 + x[2]^2 + u[1]^2)),
         FunctionPlayerCost((g, x, u, t) -> ((x[4] - 1)^2 + u[2]^2)))

player_input1 = (SVector(1), SVector(2))
player_input2 = (SVector(1), SVector(2))
player_input3 = (SVector(1), SVector(2))
player_input4 = (SVector(1), SVector(2))
player_input5 = (SVector(1), SVector(2))


