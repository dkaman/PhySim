module PhySim

using StaticArrays
using LinearAlgebra

include("Core/Engine.jl")
include("Integrators/Euler.jl")
include("Domains/Kinematics.jl")
include("UI/Dashboard.jl")

end # module PhySim
