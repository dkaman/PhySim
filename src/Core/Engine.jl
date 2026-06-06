abstract type AbstractState end
abstract type AbstractSystem end
abstract type AbstractIntegrator end
abstract type AbstractObserver end

const Vector2D = SVector{2,Float64}
const Vector3D = SVector{3,Float64}

export AbstractState, AbstractSystem, AbstractIntegrator, AbstractObserver
export Simulation
export Vector2D, Vector3D
export simulate!, evolve!, measure, sync_observer!

mutable struct Simulation{
    S<:AbstractState,
    Y<:AbstractSystem,
    I<:AbstractIntegrator,
    O<:AbstractObserver
}
    state::S
    system::Y
    integrator::I
    observer::O
    dt::Float64
    total_steps::Int
end

function Simulation(state, system, integrator, observer, total_time, dt)
    total_steps = ceil(Int, total_time / dt) + 1
    return Simulation(
        state,
        system,
        integrator,
        observer,
        dt,
        total_steps
    )
end

function simulate!(sim::Simulation)
    current_time = 0.0

    sample = measure(sim.observer, sim.state, current_time)

    measurements = Vector{typeof(sample)}(undef, sim.total_steps)

    for i in 1:sim.total_steps
        measurements[i] = measure(sim.observer, sim.state, current_time)
        evolve!(sim.integrator, sim.system, sim.state, sim.dt)
        current_time += sim.dt
    end

    return measurements
end

sync_observer!(obs::AbstractObserver, state::AbstractState) = nothing
