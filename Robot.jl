using Agents
using POMDPs
using Random
using Colors

mutable struct Robot{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Float64}
    vel::NTuple{D,Float64}
    θ::Float64
    θ̇::Float64
    vis_range::Float64
    com_range::Float64
    r::Float64 #radius of av size of the robot
    alive::Bool 
    isObstacle::Bool
    occupancy_gridmap::Matrix{Int64}
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
    r = 0.2,               # size of the agent (radius)
    extent = (100,100),    # size of the world
    begin_zone = (10,10),   # beinning zone for robots
    speed = 1.0,           # their initial velocity
    vis_range = 5.0,       # visibility range
    com_range = 5.0,       # communication range
    δt = 0.01,             # time step of the model
    seed = 1               # random seed
)

    # initialize model
    space = ContinuousSpace(extent)  # 2D euclidean space
    rng = Random.MersenneTwister(seed)  # reproducibility
    scheduler = Schedulers.fastest  # they are executed semi-synchronously,
                # in order of their indexing
    D = length(extent)  # number of dimensions
    properties = Dict(  # save the time step in the model
        :δt => δt,
        :colors => distinguishable_colors(N, [RGB(1,1,1), RGB(0,0,0)], dropseed=true),
        :seed => seed,
        :speed => speed,
    )

    model = AgentBasedModel(Robot{D}, space;
        rng = rng,
        scheduler = scheduler,
        properties = properties
    )

    # now we add the agents
    for n ∈ 1:N
        # get random position and heading
        pos = Tuple(rand(model.rng, 2)) .* begin_zone
        heading = rand(model.rng) * 2π
        vel = speed.* (
            pos[1]*cos(heading) - pos[2]*sin(heading),
            pos[1]*sin(heading) + pos[2]*cos(heading)
        )

        # initialize the agents with argument values and no heading change
        int_extent = Tuple(trunc(Int, extent[i]) for i in eachindex(extent))
        agent = Robot{D}(n, pos, vel, heading, 0.0, vis_range, com_range, r, true, false, fill(-3, int_extent))
        add_agent!(agent, pos, model)  
    end

    return model
end


function add_obstacles(model; N = 10, r = 0.5)
    D = length(model.space.extent)
    extent = model.space.extent
    for i in 1:N
        pos = Tuple(rand()*extent[i] for i in 1:D)
        obs = Robot{D}(nagents(model)+1, pos, Tuple(0 for i in 1:D), 0, 0, 0, 0, r, false, true, Array{Float64}(undef, 0 ,0))
        add_agent!(obs, pos, model)
    end
end


function gridmap_update(robot, model)
    # gridmap : -3 if unknown, -2 if occupied by obs, -1 if occupied by robot, 0 if free
    neighbours = nearby_agents_exact(robot, model, robot.vis_range) #like a lidar scan
    
    for n in neighbours    
        # test 8 points on the circle of radius r around the obstacle to fill gridmap
        neighbour_centerCell = (trunc(n.pos[1]), trunc(n.pos[2]))
        for i in 0:7
            α = i*π/4
            neighbour_extendedCell = (trunc(Int, n.pos[1]+n.r*cos(α)), trunc(Int, n.pos[2]+n.r*sin(α)))
            if neighbour_centerCell != neighbour_extendedCell
                robot.occupancy_gridmap[neighbour_extendedCell[1]+1, neighbour_extendedCell[2]+1] = n.isObstacle ? -2 : -1 
            end
        end
    end

    # for 

end



model = initialize_model(;
    N = 10,                 # number of agents
    r = 0.2,
    extent = (100,100),  # size of the world
    begin_zone = (10,10),
    speed = 1.,           # their initial velocity
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
