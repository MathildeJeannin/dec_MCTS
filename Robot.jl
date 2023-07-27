using Agents
using POMDPs
using Random
using Colors

## Version gridSpace

mutable struct Robot{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Int}
    vis_range::Float64
    com_range::Float64
    alive::Bool 
    isObstacle::Bool
    occupancy_gridmap::Matrix{Int8}
    # tree::MDP{Int64,Int64}
end

# mutable struct Obstacles{D} <: Objects
#     id::Int
#     pos::NTuple{D,Float64}
#     radius::Float64
#     isObstacle::Bool
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
        agent = Robot{D}(n, pos, vis_range, com_range, true, false, fill(-3, extent))
        add_agent!(agent, pos, model)  
    end

    return model
end


function add_obstacles(model; N = 10, extent = (100,100))
    D = length(extent)
    for i in 1:N
        pos = Tuple(rand(model.rng, 1:extent[i]) for i in 1:D)
        obs = Robot{D}(nagents(model)+1, pos, 0, 0, false, true, Array{Int64}(undef, 0 ,0))
        add_agent!(obs, pos, model)
    end
end


function gridmap_update(robot, model)
    # gridmap : -3 if unknown, -2 if occupied by obs, -1 if occupied by robot, 0 if free
    neighbours = nearby_agents(robot, model, robot.vis_range) #like a lidar scan
    
    # for n in neighbours    
    #     # test 8 points on the circle of radius r around the obstacle to fill gridmap
    #     neighbour_centerCell = (trunc(n.pos[1]), trunc(n.pos[2]))
    #     for i in 0:7
    #         α = i*π/4
    #         neighbour_extendedCell = (trunc(Int, n.pos[1]+n.r*cos(α)), trunc(Int, n.pos[2]+n.r*sin(α)))
    #         if neighbour_centerCell != neighbour_extendedCell
    #             robot.occupancy_gridmap[neighbour_extendedCell[1]+1, neighbour_extendedCell[2]+1] = n.isObstacle ? -2 : -1 
    #         end
    #     end
    # end

    # for 

end



model = initialize_model(;
    N = 10,                 # number of agents
    extent = (100,100),  # size of the world
    begin_zone = (10,10),
    vis_range = 5.,       # visibility range
    com_range = 5.,       # communication range
    δt = 0.01,             # time step of the model
    seed = 1
)

robots = allagents(model)
add_obstacles(model)

for r in robots
    if !r.isObstacle
        gridmap_update(r, model)
    end
end
