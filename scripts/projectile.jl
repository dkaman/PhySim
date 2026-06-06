using Pkg

Pkg.activate(joinpath(@__DIR__, ".."))

using PhySim
using JSON3

const gravity = Vector3D(0.0, 0.0, -9.81)

function main()
    initial_pos = Vector3D(0.0, 0.0, 0.0)
    initial_vel = Vector3D(10.0, 10.0, 100.0)
    floor_level = 0.0

    state = KinematicState(initial_pos, initial_vel, floor_level)
    sys = ConstantAcceleration(gravity)
    in = ForwardEulerIntegrator()
    obs = KinematicErrorCheckingObserver(initial_pos, initial_vel, sys.a)

    total_time = 20.0
    dt = 0.01

    sim = Simulation(state, sys, in, obs, total_time, dt)
    launch_dashboard(sim)
    readline()
end

main()
