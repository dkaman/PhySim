export launch_dashboard

function launch_dashboard(base_sim::Simulation)
    screen_size = try GLMakie.primary_resolution() catch; (1920, 1080) end
    fig = Figure(size = screen_size)

    ui_panel = fig[1:2, 1] = GridLayout(valign=:top)

    timeline_panel = fig[2, 2] = GridLayout()
    play_btn = Button(timeline_panel[1, 1], label="Play", width=80)
    time_slider = Slider(timeline_panel[1, 2], range=1:base_sim.total_steps, startvalue=1)
    time_label = Label(timeline_panel[1, 3], "0.00 s", width=60, halign=:left)

    colsize!(timeline_panel, 2, Relative(1.0))

    ax = Axis3(fig[1, 2], aspect = (1.0, 1.0, 1.0))

    colsize!(fig.layout, 1, 350)    # Lock Sidebar width to 350px
    rowsize!(fig.layout, 2, Auto()) # Lock Timeline row strictly to the height of the slider

    state_update_trigger = build_config_panel(ui_panel, base_sim.state)
    active_sim = deepcopy(base_sim)
    initial_history = simulate!(active_sim)
    history_obs = Observable(initial_history)

    render_scene!(ax, history_obs, time_slider.value)

    on(time_slider.value) do step
        time_label.text[] = string(round((step - 1) * base_sim.dt, digits=2), " s")
    end

    is_playing = Observable(false)
    on(play_btn.clicks) do _
        is_playing[] = !is_playing[]
        play_btn.label[] = is_playing[] ? "Pause" : "Play"

        if is_playing[]
            @async begin
                # Record the exact wall-clock time and starting step when "Play" is pressed
                start_wall_time = time()
                start_step = time_slider.value[]

                while is_playing[] && time_slider.value[] < length(history_obs[])
                    # Calculate true elapsed real-world time
                    elapsed_time = time() - start_wall_time

                    # Compute what frame we should be on to match real time
                    target_step = start_step + floor(Int, elapsed_time / base_sim.dt)

                    # Clamp it to the end of the array to prevent bounds errors
                    target_step = min(target_step, length(history_obs[]))

                    # Update the slider if time has advanced enough
                    if target_step > time_slider.value[]
                        set_close_to!(time_slider, target_step)
                    end

                    # Yield for ~60fps to keep the Makie event loop responsive
                    sleep(1/60)
                    yield()
                end

                # Reset button state when we hit the end
                if time_slider.value[] == length(history_obs[])
                    is_playing[] = false
                    play_btn.label[] = "Play"
                end
            end
        end
    end

    on(state_update_trigger) do new_state
        fresh_sim = deepcopy(base_sim)

        for field in fieldnames(typeof(fresh_sim.state))
            setfield!(fresh_sim.state, field, getfield(new_state, field))
        end

        sync_observer!(fresh_sim.observer, fresh_sim.state)

        history_obs[] = simulate!(fresh_sim)
    end

    on(history_obs) do _
        set_close_to!(time_slider, 1)
        autolimits!(ax)
    end

    display(fig)
end
