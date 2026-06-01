mutable struct KinematicState <: AbstractState
    r::Vector3D
    v::Vector3D
    floor_z::Float64
end

struct KinematicDerivative
    dr::Vector3D
    dv::Vector3D
end

struct ConstantAcceleration <: AbstractSystem
    a::Vector3D
end

struct KinematicErrorCheckingObserver <: AbstractObserver
    r0::Vector3D
    v0::Vector3D
    a::Vector3D
end

function get_derivative(sys::ConstantAcceleration, state::KinematicState)
    return KinematicDerivative(state.v, sys.a)
end

function apply_update!(state::KinematicState, derivative::KinematicDerivative, dt::Float64)
    next_r = state.r + (derivative.dr * dt)
    next_v = state.v + (derivative.dv * dt)
    if next_r[3] <= state.floor_z
        state.r = Vector3D(next_r[1], next_r[2], state.floor_z)
        state.v = Vector3D(0.0, 0.0, 0.0)
    else
        state.r = next_r
        state.v = next_v
    end
end

function measure(obs::KinematicErrorCheckingObserver, state::KinematicState, current_time::Float64)
    exact_r = obs.r0 + (obs.v0 * current_time) + (0.5 * obs.a  * current_time^2)
    exact_v = obs.v0 + (obs.a * current_time)

    error_r_mag = norm(state.r - exact_r)
    error_v_mag = norm(state.v - exact_v)

    return (
        time = current_time,
        sim_r = state.r,
        exact_r = exact_r,
        sim_v = state.v,
        exact_v = exact_v,
        err_r = error_r_mag,
        err_v = error_v_mag,
    )
end
