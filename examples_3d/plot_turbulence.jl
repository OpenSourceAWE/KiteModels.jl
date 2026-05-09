# SPDX-FileCopyrightText: 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end

using KiteModels, KiteUtils, LinearAlgebra, Statistics, ControlPlots

set::Settings = if haskey(ENV, "USE_V9")
    deepcopy(load_settings("system_v9.yaml"))
else
    deepcopy(load_settings("system.yaml"))
end
set.abs_tol=0.00006
set.rel_tol=0.0001
set.sample_freq = 20
set.use_turbulence = 0.3

pos_start= [60.97, 0.0, 145.42]
pos_end =  [0.08, -62.9, 143.66]
time = 0:0.1:240

# calculate v_wind_kite for pos_start and pos_end as function of time
am = AtmosphericModel(set)
upwind_dir = deg2rad(set.upwind_dir)

v_wind_start = [norm(calc_turbulent_wind(am, pos_start, upwind_dir, t)[1]) for t in time]
v_wind_end   = [norm(calc_turbulent_wind(am, pos_end,   upwind_dir, t)[1]) for t in time]

p = plotx(time, v_wind_start, v_wind_end;
          fig="v_wind_kite vs time",
          xlabel="Time [s]",
          ysize=11,
          ylabels=["v_wind_kite [m/s]", "v_wind_kite [m/s]"],
          labels=["pos_start", "pos_end"])
display(p)

wind_dir_vec = [cos(-upwind_dir - pi/2), sin(-upwind_dir - pi/2), 0.0]
v_wind_turb = map([0.0, 0.5, 1.0]) do turb
    set_turb = deepcopy(set)
    set_turb.use_turbulence = turb
    am_turb = AtmosphericModel(set_turb)
    v_vecs  = [calc_turbulent_wind(am_turb, pos_start, upwind_dir, t)[1] for t in time]
    v_mags  = norm.(v_vecs)
    mean_v  = mean(v_mags)
    # TI = std of longitudinal (along-wind) component / mean speed
    u_comp  = [dot(v, wind_dir_vec) for v in v_vecs]
    ti      = std(u_comp) / mean_v * 100
    println("use_turbulence=$(turb): mean wind = $(round(mean_v, digits=3)) m/s, turbulence intensity = $(round(ti, digits=2)) %")
    v_mags
end

p3 = plotx(time, v_wind_turb[1], v_wind_turb[2], v_wind_turb[3];
           fig="v_wind_kite vs time (pos_start, varying turbulence)",
           ysize=11,
           xlabel="Time [s]",
           ylabels=["v_wind_kite [m/s]", "v_wind_kite [m/s]", "v_wind_kite [m/s]"],
           labels=["use_turbulence=0.0", "use_turbulence=0.5", "use_turbulence=1.0"])
display(p3)
