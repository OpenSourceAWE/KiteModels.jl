# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT
#
# Loads and plots the frequency response spectrum of the kite system
# from previously computed data files (.jld2). Visualizes the angle-of-attack
# gain over a range of excitation frequencies for different configurations.

using KiteModels, LinearAlgebra

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
using MakieControlPlots, DSP, JLD2, StatsBase

#if !@isdefined Spectrum begin
    mutable struct Spectrum
        name::String
        cmq::Float64
        v_wind::Float64
        f_ex::Vector{Float64}
        aoa_eff::Vector{Float64}
    end
#end

function plot_spectrum3(name)
    todb(mag) = 20 * log10(mag)
    filepath = "data/" * name * ".jld2"
    if !isfile(filepath)
        @warn "File not found: $filepath"
        @info "Hint: run the example calc_spectrum first"
        return
    end
    spectrum = jldopen(filepath) do file
        read(file, "spectrum")
    end
    f_min = 1.5 # Hz
    f_max = 4.0 # Hz
    f_ex = Float64[]
    aoa_eff = Float64[]
    for i in 1:length(spectrum.f_ex)
        if spectrum.f_ex[i] < f_min
            continue
        end
        if spectrum.f_ex[i] > f_max
            break
        end
        push!(f_ex, spectrum.f_ex[i])
        push!(aoa_eff, spectrum.aoa_eff[i])
    end
    display(plot(f_ex, todb.(aoa_eff); xlabel="f_ex [Hz]", ylabel="AOA amplitude [dB°]", xscale=:log10, grid=true, label=name, fig="spectrum", xticks=[1.5, 2.0, 3.0, 4.0], xlims=(f_min, f_max)))
end

plot_spectrum3("spectrum2_8.0_-0.0")
reactivate_host_app()
