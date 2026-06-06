export ForwardEulerIntegrator

struct ForwardEulerIntegrator <: AbstractIntegrator end

function evolve!(in::ForwardEulerIntegrator, sys::AbstractSystem, state::AbstractState, dt::Float64)
    derivative = get_derivative(sys, state)
    apply_update!(state, derivative, dt)
end
