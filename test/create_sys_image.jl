# SPDX-FileCopyrightText: 2022, 2024, 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

# activate the test environment if needed
using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
    Pkg.resolve()
    Pkg.instantiate()
end
@info "Loading packages ..."
using AtmosphericModels, Colors, ControlSystemsBase, DSP, Dierckx, DiscretePIDs,
    DocStringExtensions, JLD2, KitePodModels, KiteUtils, KiteViewers, LinearAlgebra, NLsolve,
    NonlinearSolve, OrdinaryDiffEqBDF, OrdinaryDiffEqCore, OrdinaryDiffEqNonlinearSolve,
    Parameters, REPL, Rotations, StaticArrays, StatsBase, Sundials, WinchModels
using BenchmarkTools, Documenter, PackageCompiler

@info "Creating sysimage ..."
push!(LOAD_PATH,joinpath(pwd(),"src"))

GC.gc(true)
let mem = Sys.free_memory() / 1024^2
    @info "Free memory: $(round(mem; digits=1)) MB"
    if haskey(ENV, "JULIA_IMAGE_THREADS")
        @info "JULIA_IMAGE_THREADS: $(ENV["JULIA_IMAGE_THREADS"])"
    else
        @info "JULIA_IMAGE_THREADS not defined!"
    end
end

PackageCompiler.create_sysimage(
    [:Dierckx, :StaticArrays, :Parameters, :NLsolve, :DocStringExtensions, :Sundials, :KiteUtils, 
     :KitePodModels, :AtmosphericModels, :OrdinaryDiffEqCore, :OrdinaryDiffEqBDF, :WinchModels,
     :OrdinaryDiffEqNonlinearSolve,
     :JLD2, :Colors, :REPL, :NonlinearSolve, :DSP, :DiscretePIDs, 
     :ControlSystemsBase, :KiteViewers, :Rotations];
    sysimage_path="kps-image_tmp.so",
    include_transitive_dependencies=true,
    precompile_execution_file=joinpath("test", "test_for_precompile.jl")
)