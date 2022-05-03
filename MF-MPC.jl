using JuMP, Ipopt, EAGO
using StaticArrays
using Statistics
using Infiltrator
####################################
# Write MPC for unicycle Model - Done
# Write MPC for all the agents to avoid each other - Done
# Extend to Robust MPC
####################################

model = Model(Ipopt.Optimizer)
# model = Model(EAGO.Optimizer)

# model = Model(NLopt.Optimizer)
# set_optimizer_attribute(model, "algorithm", :AUGLAG)
# set_optimizer_attribute(model, "local_optimizer", :LD_LBFGS)

function bicycle_dynamics_solver(prev_state, controls, delta_t)
    new_state = [0.0,0.0,0.,0.]
    l = 3
    new_state[1] = prev_state[1] + prev_state[4]*cos(prev_state[3])
    new_state[2] = prev_state[2] + prev_state[4]*sin(prev_state[3])
    new_state[3] = prev_state[3] + controls[1]
    new_state[4] = prev_state[4] + controls[2]
    return new_state
end

d = 10
n_agents = (5) #agent states for Unicycle model: 4* number of agents you want to take 
n_states = 4*n_agents
current_state = [-10,-10,0,0,10,-10,0,0,50,50,0,0,20,20,0,0,30,30,0,0]
goal = [10,10,0,0,-10,10,0,0,60,60,0,0,30,30,0,0,40,40,0,0]
obstacle = [3,4]

# Storing position of velocity and position seperation representation
state_mid = Int(round(n_states/2))
state_quater = Int(round(n_states/4))
@variables model begin
s[1:n_states, 1:d]
-pi/4 <= angular_vel[1:n_agents,1:d] <= pi/4
-3 <= vel[1:n_agents,1:d] <= 3
end

# velocity update
@NLconstraint(model, [i=2:d,j=1:4:n_states], s[j,i] == s[j,i-1] + s[j+3,i-1]*cos(s[j+2,i-1]))
# position update
@NLconstraint(model, [i=2:d,j=2:4:n_states], s[j,i] == s[j,i-1] + s[j+2,i-1]*sin(s[j+1,i-1]))
# angular velocity update 
@constraint(model, [i=2:d, j=3:4:n_states], s[j, i] ==  s[j, i-1]+ angular_vel[Int(floor(j/4))+1,i-1])
# velocity update
@constraint(model, [i=2:d, j=4:4:n_states], s[j, i] ==  s[j, i-1]+ vel[Int(round(j/4)),i-1])

for constraint_cnt = 1:4:n_states
# no collision update
    for j = 5:4:n_states
        if j == constraint_cnt
            continue
        end
        print("X coord ",j,"\t",constraint_cnt,"\n")
        print("Y coord ",j+1,"\t",constraint_cnt+1,"\n")
        @NLconstraint(model, [i=2:d], ((s[j,i-1] - s[constraint_cnt,i-1])^2 + (s[j+1,i-1] - s[constraint_cnt+1,i-1])^2) >=  4)
    end
end

print(state_mid, "\n")
# initial condition
@constraint(model, s[1:n_states,1] .== current_state)
# obstacle
# @constraint(model, [i=1:d], sum((s[1:2,i] - obstacle).^2) ≥ 4)

@objective(model, Min, 100*sum((s[:,d] - goal).^2) + sum(angular_vel.^2) + sum(vel.^2))



optimize!(model)
angular_vel_val = value.(angular_vel)
vel_val = value.(vel)
states = value.(s)
action_1 = SVector{n_agents,Float64}(angular_vel_val[:,1])
action_2 = SVector{n_agents,Float64}(vel_val[:,1])

for _ = 0:delta_T:T
    global angular_vel, vel
    optimize!(model)
    angular_vel_val = value.(angular_vel)
    vel_val = value.(vel)
    states = value.(s)
    action_1 = SVector{n_agents,Float64}(angular_vel_val[:,1])
    action_2 = SVector{n_agents,Float64}(vel_val[:,1])

    
end
