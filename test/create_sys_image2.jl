# SPDX-FileCopyrightText: 2022, 2024, 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if ! ("PackageCompiler" ∈ keys(Pkg.project().dependencies))
    @info "Installing PackageCompiler ..."
    Pkg.add("PackageCompiler")
end
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    @info "Installing MakieControlPlots ..."
    Pkg.add("MakieControlPlots")
end
@info "Loading packages ..."
using KiteUtils, KitePodModels, KiteModels, MakieControlPlots
using PackageCompiler

@info "Creating sysimage ..."
push!(LOAD_PATH,joinpath(pwd(),"src"))

PackageCompiler.create_sysimage(
    [:KiteUtils, :KitePodModels, :KiteModels, :MakieControlPlots];
    sysimage_path="kps-image_tmp.so",
    include_transitive_dependencies=true,
    precompile_execution_file=joinpath("test", "test_for_precompile.jl"),
    # PackageCompiler 2.4.1+ defaults to the multi-target CPU list of the Julia binary on
    # Julia 1.13+, which compiles every function several times and needs more than twice
    # the memory and time (JuliaLang/julia#63141); build for this machine only, as on 1.12
    cpu_target="native"
)
