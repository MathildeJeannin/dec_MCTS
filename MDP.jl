using POMDPs: POMDP, MDP
using POMDPTools: Deterministic, Uniform, SparseCat
using QuickPOMDPs: QuickMDP
using QMDP
using MCTS
using POMDPModels

struct robotMDP <: MDP{Tuple{Int8,Int8,Bool}, Tuple{Int8,Int8}}

end

extent = (100,100)
S = Array{Tuple{Int,Int,Bool}}(undef, 0)

for i in 1:extent[1]
    for j in extent[2]
        for isObs in [true,false]
            push!(S, (i,j,isObs))
        end
    end
end

A = Array{Tuple{Int,Int}}(undef, 0)
for i in -1:1
    for j in -1:1
        push!(A, (i,j))
    end
end

states(m::robotMDP) = S
actions(m::robotMDP) = A
stateindex(m::robotMDP, s) = m.indices[s]
actionindex(m::robotMDP, a) = m.indices[a]

# function transition(m::robotMDP, s, a)
#     if s[3] = true
#         return 0
#     elseif s[2]==-1 || s[1:2] == (0,1)
#         return 0.1
# end