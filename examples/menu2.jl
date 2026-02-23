# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT
using REPL.TerminalMenus

options =  if haskey(ENV, "USE_V9")
          ["plot_cl_cd_plate = include(\"plot_cl_cd_plate.jl\")",
           "plot_side_cl = include(\"plot_side_cl.jl\")",
           "steering_test_4p = include(\"steering_test_4p.jl\")",
           "plot_parking_test = include(\"plot_parking_test.jl\")",
           "calculate_rot_inertia = include(\"calculate_rotational_inertia.jl\")",
           "show_kite = include(\"../examples_3d/show_kite.jl\")",
           "parking_4p = include(\"../examples_3d/parking_4p.jl\")",
           "auto_parking_4p = include(\"../examples_3d/auto_parking_4p.jl\")",
           "parking_wind_dir = include(\"../examples_3d/parking_wind_dir.jl\")",
           "build_docu = include(\"../scripts/build_docu.jl\")",
           "quit"]
else
          ["plot_cl_cd_plate = include(\"plot_cl_cd_plate.jl\")",
           "plot_side_cl = include(\"plot_side_cl.jl\")",
           "steering_test_4p = include(\"steering_test_4p.jl\")",
           "calculate_rot_inertia = include(\"calculate_rotational_inertia.jl\")",
           "show_kite = include(\"../examples_3d/show_kite.jl\")",
           "parking_4p = include(\"../examples_3d/parking_4p.jl\")",
           "auto_parking_4p = include(\"../examples_3d/auto_parking_4p.jl\")",
           "parking_wind_dir = include(\"../examples_3d/parking_wind_dir.jl\")",
           "build_docu = include(\"../scripts/build_docu.jl\")",
           "quit"]
end

function extra_menu()
    active = true
    while active
        menu = RadioMenu(options, pagesize=12)
        choice = request("\nChoose function to execute or `q` to quit: ", menu)

        if choice != -1 && choice != length(options)
            eval(Meta.parse(options[choice]))
        else
            println("Left menu. Press <ctrl><d> to quit Julia!")
            active = false
        end
    end
end

extra_menu()