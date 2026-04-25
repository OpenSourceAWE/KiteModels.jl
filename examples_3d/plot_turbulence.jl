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
wind_dir = -deg2rad(set.upwind_dir) - pi/2+deg2rad(90)

v_wind_start = [norm(calc_turbulent_wind(am, pos_start, wind_dir, t)[1]) for t in time]
v_wind_end   = [norm(calc_turbulent_wind(am, pos_end,   wind_dir, t)[1]) for t in time]

p = plotx(time, v_wind_start, v_wind_end;
          fig="v_wind_kite vs time",
          xlabel="Time [s]",
          ylabels=["v_wind_kite [m/s]", "v_wind_kite [m/s]"],
          labels=["pos_start", "pos_end"])
display(p)

v_wind_turb = map([0.0, 0.5, 1.0]) do turb
    am.set.use_turbulence = turb
    v = [norm(calc_turbulent_wind(am, pos_start, wind_dir, t)[1]) for t in time]
    mean_v = mean(v)
    std_v  = std(v)
    ti     = std_v / mean_v * 100
    println("use_turbulence=$(turb): mean wind = $(round(mean_v, digits=3)) m/s, turbulence intensity = $(round(ti, digits=2)) %")
    v
end
am.set.use_turbulence = set.use_turbulence  # restore original value

p3 = plotx(time, v_wind_turb[1], v_wind_turb[2], v_wind_turb[3];
           fig="v_wind_kite vs time (pos_start, varying turbulence)",
           xlabel="Time [s]",
           ylabels=["v_wind_kite [m/s]", "v_wind_kite [m/s]", "v_wind_kite [m/s]"],
           labels=["use_turbulence=0.0", "use_turbulence=0.5", "use_turbulence=1.0"])
display(p3)
