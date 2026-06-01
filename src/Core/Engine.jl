abstract type AbstractState end
abstract type AbstractSystem end
abstract type AbstractIntegrator end
abstract type AbstractObserver end

const Vector2D = SVector{2, Float64}
const Vector3D = SVector{3, Float64}

function simulate!(
    state::AbstractState,
    sys::AbstractSystem,
    in::AbstractIntegrator,
    obs::AbstractObserver,
    total_time::Float64,
    dt::Float64
)
    measurements = []

    current_time = 0.0
    steps = Int(total_time / dt)

    for _ in 1:steps
        val = measure(obs, state, current_time)
        push!(measurements, val)
        evolve!(in, sys, state, dt)
        current_time += dt
    end

    return measurements
end
