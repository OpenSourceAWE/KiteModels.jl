using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end

using KiteModels, KiteUtils, LinearAlgebra, ControlPlots

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
time = 0:0.1:120

# calculate v_wind_kite for pos_start and pos_end as function of time
am = AtmosphericModel(set)
wind_dir = -deg2rad(set.upwind_dir) - pi/2+deg2rad(90)

v_wind_start = [norm(calc_turbulent_wind(am, pos_start, wind_dir, t)[1]) for t in time]
v_wind_end   = [norm(calc_turbulent_wind(am, pos_end,   wind_dir, t)[1]) for t in time]

p = plotx(time, v_wind_start, v_wind_end;
          fig="v_wind_kite vs time",
          xlabel="Time [s]",
          ylabels=["v_wind_kite [m/s]", "v_wind_kite [m/s]"],
          labels=["pos_start", "pos_end"])
display(p)

wind_dirs = range(-pi, pi, length=360)
v_wind_start_dir = [norm(calc_turbulent_wind(am, pos_start, wd, 0.0)[1]) for wd in wind_dirs]
v_wind_end_dir   = [norm(calc_turbulent_wind(am, pos_end,   wd, 0.0)[1]) for wd in wind_dirs]

p2 = plotx(rad2deg.(wind_dirs), v_wind_start_dir, v_wind_end_dir;
           fig="v_wind_kite vs wind_dir",
           xlabel="wind_dir [°]",
           ylabels=["v_wind_kite [m/s]", "v_wind_kite [m/s]"],
           labels=["pos_start", "pos_end"])
display(p2)
