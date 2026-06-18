# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT
using REPL.TerminalMenus
using KiteModels: set_default_turbulence

actions = if haskey(ENV, "USE_V9")
    [
        ("set_default_turbulence", nothing),
        ("plot_cl_cd_plate", "plot_cl_cd_plate.jl"),
        ("plot_side_cl", "plot_side_cl.jl"),
        ("steering_test_4p", "steering_test_4p.jl"),
        ("plot_parking_test", "plot_parking_test.jl"),
        ("calculate_rot_inertia", "calculate_rotational_inertia.jl"),
        ("show_kite", "../examples_3d/show_kite.jl"),
        ("parking_4p", "../examples_3d/parking_4p.jl"),
        ("auto_parking_4p", "../examples_3d/auto_parking_4p.jl"),
        ("parking_wind_dir", "../examples_3d/parking_wind_dir.jl"),
        ("build_docu", "../scripts/build_docu.jl")
    ]
else
    [
        ("set_default_turbulence", nothing),
        ("plot_cl_cd_plate", "plot_cl_cd_plate.jl"),
        ("plot_side_cl", "plot_side_cl.jl"),
        ("steering_test_4p", "steering_test_4p.jl"),
        ("calculate_rot_inertia", "calculate_rotational_inertia.jl"),
        ("show_kite", "../examples_3d/show_kite.jl"),
        ("parking_4p", "../examples_3d/parking_4p.jl"),
        ("auto_parking_4p", "../examples_3d/auto_parking_4p.jl"),
        ("parking_wind_dir", "../examples_3d/parking_wind_dir.jl"),
        ("build_docu", "../scripts/build_docu.jl")
    ]
end

options = [path === nothing ? name : "$(name) = include(\"$(path)\")" for (name, path) in actions]
push!(options, "quit")

function run_sandboxed(script_path::String)
    abs_path = normpath(joinpath(@__DIR__, script_path))
    module_name = Symbol("MenuSandbox_", time_ns())
    sandbox = Module(module_name)
    Core.eval(sandbox, :(include(path::AbstractString) = Base.include($sandbox, path)))
    Base.include(sandbox, abs_path)
    nothing
end

function extra_menu()
    active = true
    while active
        menu = RadioMenu(options, pagesize=12)
        choice = request("\nChoose function to execute or `q` to quit: ", menu)

        if choice != -1 && choice != length(options)
            _, script_path = actions[choice]
            if script_path === nothing
                set_default_turbulence()
            else
                run_sandboxed(script_path)
            end
        else
            println("Left menu. Press <ctrl><d> to quit Julia!")
            active = false
        end
    end
end

extra_menu()