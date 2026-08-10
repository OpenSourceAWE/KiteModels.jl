# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

# Plot the lift and drag coefficients as function of angle of attack
# of any of the plates of the kite.

using KiteModels
using KiteUtils: Settings, load_settings

set::Settings = if haskey(ENV, "USE_V9")
    deepcopy(load_settings("system_v9.yaml"))
else
    deepcopy(load_settings("system.yaml"))
end

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using LaTeXStrings, MakieControlPlots

set.v_wind = 14.0 # 25
kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)

function plot_cl_cd(alpha)
    cl = zeros(length(alpha))
    cd = zeros(length(alpha))
    for (i, alpha) in pairs(ALPHA)
        cl[i] = kps4.calc_cl(alpha)
        cd[i] = kps4.calc_cd(alpha)
    end
    display(plot(ALPHA, [cl, cd]; xlabel=L"\mathrm{AoA}~\alpha", ylabel="CL, CD", labels=["CL", "CD"], fig="CL_CD", xticks=-10:5:20, xsize=18))
    reactivate_host_app()
    display(plot(ALPHA, [cl./cd]; xlabel=L"\mathrm{AoA}~\alpha", ylabel="LoD", labels=["LoD"], fig="LoD", xticks=-10:5:20, xsize=18))
    reactivate_host_app()
end

ALPHA = -10:0.1:20
plot_cl_cd(ALPHA)

cl1 = kps4.calc_cl(-5)
cl2 = kps4.calc_cl(12)
dcl_over_dalpha= (cl2-cl1)/deg2rad(12+5)
println("dCL/dalpha = ", round(dcl_over_dalpha, digits=3))