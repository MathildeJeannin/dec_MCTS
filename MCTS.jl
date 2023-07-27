using POMDPs: POMDP, MDP
using POMDPTools: Deterministic, Uniform, SparseCat
using QuickPOMDPs: QuickMDP
using QMDP
using MCTS
using POMDPModels

# m = QuickMDP(
#     states = 1:25,
#     actions = ["up", "right", "left", "down", "up_right", "up_left", "down_right", "down_left"],
#     discount = 0.95,

#     transition = function (s, a)
#         return Uniform(["up", "right", "left", "down", "up_right", "up_left", "down_right", "down_left"])
#     end,

#     reward = function(s, a)
#         actionsNumber = Dict("up"=>-5, "right"=>1, "left"=>-1, "down"=>5)
#         an = actionsNumber[a]
#         sp = s + an 
#         if sp < 1 || sp > 25 || s%5 == 0 && an == 1 || (s-1)%5 == 0 && an == -1
#             return -1.0
#         else 
#             return 1.0 
#         end
#     end,

#     initialstate = Uniform(1:25),
# );


# solver = MCTSSolver(n_iterations=50,
#     depth=20,
#     exploration_constant=10.0)
# policy = solve(solver, m)
# a = action(policy, 1)

