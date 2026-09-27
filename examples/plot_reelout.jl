# Copyright (c) 2024 Uwe Fechner
# SPDX-License-Identifier: MIT
#
# Loads the log file written by reel_out_4p_torque_control.jl and
# recreates its reel-out speed/force/elevation/heading/wind plot from
# the saved data, without re-running the simulation.

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end

using KiteModels, LinearAlgebra
using MakieControlPlots

LOGFILE = "reel_out_4p_torque_control"

log = load_log(LOGFILE)
sl = log.syslog

v_time      = sl.time
v_speed     = first.(sl.v_reelout)
v_force     = first.(sl.winch_force)
v_elevation = rad2deg.(sl.elevation)
v_heading   = rad2deg.(wrap2pi.(sl.heading))
v_wind_kite = norm.(sl.v_wind_kite)

p = plotx(v_time, v_speed, v_force, v_elevation, v_heading, v_wind_kite;
    ylabels=["v_reelout  [m/s]", "tether_force [N]", "elevation [deg]", "heading [deg]", "wind at kite [m/s]"],
    fig="winch")
display(p)
reactivate_host_app()
nothing
