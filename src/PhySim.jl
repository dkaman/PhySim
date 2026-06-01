module PhySim

using StaticArrays
using LinearAlgebra

export AbstractState, AbstractSystem, AbstractIntegrator, AbstractObserver
export KinematicState, KinematicDerivative, ConstantAcceleration, KinematicErrorCheckingObserver
export ForwardEulerIntegrator
export simulate!, evolve!, measure
export Vector2D, Vector3D

include("Core/Engine.jl")
include("Integrators/Euler.jl")
include("Domains/Kinematics.jl")

end # module PhySim
