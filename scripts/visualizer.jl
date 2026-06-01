using Pkg

Pkg.activate(joinpath(@__DIR__, ".."))

using JSON3
using GLMakie

function get_axis_limits(components, min_span = 10.0)
    v_min, v_max = minimum(components), maximum(components)
    span = v_max - v_min

    if span < min_span
        center = (v_min + v_max) / 2
        return (center - min_span/2, center + min_span/2)
    else
        padding = span * 0.05
        return (v_min - padding, v_max + padding)
    end
end

function main()
    raw_data = open(io -> JSON3.read(io), joinpath(@__DIR__, "output.json"))
    sim_points = Point3f[Point3f(d.sim_r[1], d.sim_r[2], d.sim_r[3]) for d in raw_data]
    exact_points = Point3f[Point3f(d.exact_r[1], d.exact_r[2], d.exact_r[3]) for d in raw_data]

    xs = [p[1] for p in sim_points]
    ys = [p[2] for p in sim_points]
    zs = [p[3] for p in sim_points]

    fig = Figure(size = (1000, 800))

    ax = Axis3(
        fig[1, 1],
        title = "3D Trajectory Analysis",
        xlabel = "X Position (m)",
        ylabel = "Y Position (m)",
        zlabel = "Z Altitude (m)",
        aspect = :data,
        limits = (get_axis_limits(xs), get_axis_limits(ys), get_axis_limits(zs)) # <-- FORCED LIMITS
    )

    lines!(ax, sim_points, color = :blue, linewidth = 4, label = "Simulated")
    lines!(ax, exact_points, color = :red, linewidth = 2, linestyle = :dash, label = "Exact")
    axislegend(ax, position = :rt)

    display(fig)
    readline()
end

main()
