include("Robot.jl")

using Agents
using GLMakie
using GraphMakie
using Graphs
using InteractiveDynamics
using DataStructures: CircularBuffer
using Random
using Makie.Colors


model = initialize_model(;
    N = 10,                 # number of agents
    extent = (100,100),  # size of the world
    begin_zone = (10,10),
    vis_range = 5.,       # visibility range
    com_range = 5.,       # communication range
    δt = 0.01,             # time step of the model
    seed = 1
)

add_obstacles(model)

abmvideo("test.mp4", model, agent_step!; framerate=4, frames=20)