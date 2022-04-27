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

d = 10
n_agents = (5) #agent states for Unicycle model: 4* number of agents you want to take 
n_states = 4*n_agents
current_state = [-10,-10,10,-10,50,50,20,20,30,30,0,0,0,0,0,0,0,0,0,0]
goal = [10,10,-10,10,60,60,30,30,40,40,0,0,0,0,0,0,0,0,0,0]
obstacle = [3,4]

@variables model begin
s[1:n_states, 1:d]
-10 ≤ a[1:Int(round(n_states/2)),1:d] ≤ 10
end

# velocity update
@constraint(model, [i=2:d,j=1:Int(round(n_states/2))], s[j,i] == s[j,i-1] + a[j,i-1])
# position update
@constraint(model, [i=2:d,j=1:Int(round(n_states/2))], s[j,i] == s[j,i-1] + s[Int(round((n_states/2)))+j,i-1])

for constraint_cnt = 1:2:Int(round(n_states/2))-2
# no collision update
    for j = 1:2:Int(round(n_states/2))-2
        if j == constraint_cnt+2
            continue
        end
        print("X coord ",j,"\t",constraint_cnt+2,"\n")
        print("Y coord ",j+1,"\t",constraint_cnt+3,"\n")
        @NLconstraint(model, [i=2:d], ((s[j,i-1] - s[constraint_cnt+2,i-1])^2 + (s[j+1,i-1] - s[constraint_cnt+3,i-1])^2) >=  4)
    end
end

print(Int(round(n_states/2)), "\n")
# initial condition
@constraint(model, s[1:n_states,1] .== current_state)
# obstacle
@constraint(model, [i=1:d], sum((s[1:2,i] - obstacle).^2) ≥ 4)
@objective(model, Min, 100*sum((s[:,d] - goal).^2) + sum(a.^2))
optimize!(model)
@time for counter = 1:10
    global states = value.(s)
    global current_state = states[:,counter]
    @time optimize!(model)
end
action = value.(a)
states = value.(s)
@infiltrate
@show states