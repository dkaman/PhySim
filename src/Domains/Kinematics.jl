using GLMakie

export KinematicState, KinematicDerivative, ConstantAcceleration, KinematicErrorCheckingObserver

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

mutable struct KinematicErrorCheckingObserver <: AbstractObserver
    r0::Vector3D
    v0::Vector3D
    a::Vector3D
end

function get_derivative(sys::ConstantAcceleration, state::KinematicState)
    return KinematicDerivative(state.v, sys.a)
end

function sync_observer!(obs::KinematicErrorCheckingObserver, state::KinematicState)
    obs.r0 = state.r
    obs.v0 = state.v
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

function build_config_panel(ui_layout, initial_state::KinematicState)
    Label(ui_layout[1, 1:4], "Simulation Parameters", font=:bold)

    # --- Position Vector Row ---
    Label(ui_layout[2, 1], "r₀ (x,y,z)", halign=:left)
    rx = Textbox(ui_layout[2, 2], stored_string = string(initial_state.r[1]), width=50)
    ry = Textbox(ui_layout[2, 3], stored_string = string(initial_state.r[2]), width=50)
    rz = Textbox(ui_layout[2, 4], stored_string = string(initial_state.r[3]), width=50)

    # --- Velocity Vector Row ---
    Label(ui_layout[3, 1], "v₀ (x,y,z)", halign=:left)
    vx = Textbox(ui_layout[3, 2], stored_string = string(initial_state.v[1]), width=50)
    vy = Textbox(ui_layout[3, 3], stored_string = string(initial_state.v[2]), width=50)
    vz = Textbox(ui_layout[3, 4], stored_string = string(initial_state.v[3]), width=50)

    # --- Floor Row ---
    Label(ui_layout[4, 1], "Floor Z (m)", halign=:left)
    floor_input = Textbox(ui_layout[4, 2:4], stored_string = string(initial_state.floor_z))

    # --- Submit Button ---
    update_btn = Button(ui_layout[5, 1:4], label = "Save & Resimulate", buttoncolor = :lightgreen)

    rowgap!(ui_layout, 10)
    trigger = Observable(initial_state)

    on(update_btn.clicks) do _
        # Parse the 3D grid back into discrete Vector3D objects
        new_r = Vector3D(
            parse(Float64, rx.displayed_string[]),
            parse(Float64, ry.displayed_string[]),
            parse(Float64, rz.displayed_string[])
        )
        new_v = Vector3D(
            parse(Float64, vx.displayed_string[]),
            parse(Float64, vy.displayed_string[]),
            parse(Float64, vz.displayed_string[])
        )
        new_floor = parse(Float64, floor_input.stored_string[])

        trigger[] = KinematicState(new_r, new_v, new_floor)
    end

    return trigger
end

function render_scene!(ax::Axis3, history_obs::Observable, current_step_obs::Observable)
    # Extract the full background simulated path (updates only on resimulation)
    full_sim_path = lift(history_obs) do hist
        return Point3f[Point3f(d.sim_r[1], d.sim_r[2], d.sim_r[3]) for d in hist]
    end

    # Active Blue Trail: Truncated from step 1 to the current slider position
    sim_trail = lift(history_obs, current_step_obs) do hist, step
        # Defensive check: Prevents out-of-bounds errors during asynchronous updates
        idx = clamp(step, 1, length(hist))
        return Point3f[Point3f(d.sim_r[1], d.sim_r[2], d.sim_r[3]) for d in hist[1:idx]]
    end

    # Active Red Dash Trail: Analytical truth up to the current slider position
    exact_trail = lift(history_obs, current_step_obs) do hist, step
        idx = clamp(step, 1, length(hist))
        return Point3f[Point3f(d.exact_r[1], d.exact_r[2], d.exact_r[3]) for d in hist[1:idx]]
    end

    # Moving Particle: A single-element array containing the active coordinate
    particle_pos = lift(history_obs, current_step_obs) do hist, step
        idx = clamp(step, 1, length(hist))
        p = hist[idx].sim_r
        return [Point3f(p[1], p[2], p[3])]
    end

    # Launch Point: Always matches the first index of the active dataset
    launch_pos = lift(history_obs) do hist
        p = hist[1].sim_r
        return [Point3f(p[1], p[2], p[3])]
    end

    # Render a faint ghost trail of the overall arc to give visual context
    lines!(ax, full_sim_path, color = (:blue, 0.12), linewidth = 1)

    # Render the bold historical lines drawn up to the current scrubber moment
    lines!(ax, sim_trail, color = :blue, linewidth = 4, label = "Simulated (Euler)")
    lines!(ax, exact_trail, color = :red, linewidth = 2, linestyle = :dash, label = "Analytical")

    # Render a static green marker at the origin point
    scatter!(ax, launch_pos, color = :green, markersize = 12, marker = :circle, label = "Launch")

    # Render the active physical object as a prominent magenta orb
    scatter!(ax, particle_pos, color = :magenta, markersize = 20, marker = :circle)

    # This prevents duplicate legends from rendering if the axis layout updates
    axislegend(ax, position = :rt, framevisible = true, bgcolor = (:white, 0.85))

    return ax
end
