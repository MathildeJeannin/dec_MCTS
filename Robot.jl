using Agents
using POMDPs
using Random
using Colors

include("MCTS.jl")

## Version gridSpace

mutable struct Robot{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Int}
    vis_range::Float64
    com_range::Float64
    alive::Bool 
    isObstacle::Bool
    occupancy_gridmap::Matrix{Int8}
    tree::MDP{Int64,Int64}
end

# struct tree <: MDP{Int, Int}
# end


function initialize_model(;
    N = 10,                # number of agents
    extent = (100,100),    # size of the world
    begin_zone = (10,10),   # beinning zone for robots
    vis_range = 5.0,       # visibility range
    com_range = 5.0,       # communication range
    δt = 0.01,             # time step of the model
    seed = 1               # random seed
)

    # initialize model
    space = GridSpace(extent; metric = :euclidean)  # 2D euclidean space
    rng = Random.MersenneTwister(seed)  # reproducibility
    scheduler = Schedulers.fastest  # they are executed semi-synchronously,
                # in order of their indexing
    D = length(extent)  # number of dimensions
    properties = Dict(  # save the time step in the model
        :δt => δt,
        :colors => distinguishable_colors(N, [RGB(1,1,1), RGB(0,0,0)], dropseed=true),
        :seed => seed,
    )

    model = AgentBasedModel(Robot{D}, space;
        rng = rng,
        scheduler = scheduler,
        properties = properties
    )

    # now we add the agents
    for n ∈ 1:N
        # get random position in a 10x10 beginning zone
        pos = Tuple(rand(model.rng, 1:begin_zone[i]) for i in 1:D)
        # initialize the agents with argument values and no heading change
        # tree = MDP{}

        agent = Robot{D}(n, pos, vis_range, com_range, true, false, fill(-2, extent))
        add_agent!(agent, pos, model)  
    end

    return model
end


function add_obstacles(model; N = 10, extent = (100,100))
    D = length(extent)
    for i in 1:N
        pos = Tuple(rand(model.rng, 1:extent[i]) for i in 1:D)
        obs = Robot{D}(nagents(model)+1, pos, 0, 0, false, true, Array{Int8}(undef, 0 ,0))
        add_agent!(obs, pos, model)
    end
end


function gridmap_update(robot, model)
    # gridmap : -2 if unknown, -1 if occupied, 0 if free
    scan = nearby_positions(robot.pos, model, robot.vis_range)
    neighbours = nearby_agents(robot, model, robot.vis_range) 
    #like a lidar scan
    pos_neighbours = Array{Tuple{Int, Int}}(undef, 0)
    for n in neighbours
        push!(pos_neighbours, (n.pos[1],n.pos[2]))
    end

    for cell in scan
        if (cell[1],cell[2]) in pos_neighbours
            robot.occupancy_gridmap[cell[1],cell[2]] = -1
        else
            robot.occupancy_gridmap[cell[1],cell[2]] = 0
        end
    end
end



function MCTS(robot, model)

end



function agent_step!(robot, model)
    if !robot.isObstacle
        gridmap_update(robot, model)
        robot.pos = (robot.pos[1]+1, robot.pos[2])
    end
end


# model = initialize_model(;
#     N = 10,                 # number of agents
#     extent = (100,100),  # size of the world
#     begin_zone = (10,10),
#     vis_range = 5.,       # visibility range
#     com_range = 5.,       # communication range
#     δt = 0.01,             # time step of the model
#     seed = 1
# )

# robots = allagents(model)
# add_obstacles(model)

# for r in robots
#     if !r.isObstacle
#         gridmap_update(r, model)
#     end
# end
