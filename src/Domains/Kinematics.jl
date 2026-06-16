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
        safe_parse(s) = something(tryparse(Float64, s), 0.0)
        # Parse the 3D grid back into discrete Vector3D objects
        new_r = Vector3D(
            safe_parse(rx.displayed_string[]),
            safe_parse(ry.displayed_string[]),
            safe_parse(rz.displayed_string[])
        )
        new_v = Vector3D(
            safe_parse(vx.displayed_string[]),
            safe_parse(vy.displayed_string[]),
            safe_parse(vz.displayed_string[])
        )

        new_floor = safe_parse(floor_input.stored_string[])
        trigger[] = KinematicState(new_r, new_v, new_floor)
    end

    return trigger
end

function render_scene!(ax::Axis3, history_obs::Observable, current_step_obs::Observable)
    full_sim_path = @lift [Point3f(d.sim_r) for d in $history_obs]

    sim_trail = @lift begin
        idx = clamp($current_step_obs, 1, length($history_obs))
        [Point3f(d.sim_r) for d in @view $history_obs[1:idx]]
    end

    exact_trail = @lift begin
        idx = clamp($current_step_obs, 1, length($history_obs))
        [Point3f(d.exact_r) for d in @view $history_obs[1:idx]]
    end

    particle_pos = @lift begin
        idx = clamp($current_step_obs, 1, length($history_obs))
        [Point3f($history_obs[idx].sim_r)]
    end

    launch_pos = @lift [Point3f($history_obs[1].sim_r)]

    lines!(ax, full_sim_path, color = (:blue, 0.12), linewidth = 1)
    lines!(ax, sim_trail, color = :blue, linewidth = 4, label = "Simulated (Euler)")
    lines!(ax, exact_trail, color = :red, linewidth = 2, linestyle = :dash, label = "Analytical")
    scatter!(ax, launch_pos, color = :green, markersize = 12, marker = :circle, label = "Launch")
    scatter!(ax, particle_pos, color = :magenta, markersize = 20, marker = :circle)
    axislegend(ax, position = :rt, framevisible = true, bgcolor = (:white, 0.85))

    return ax
end
