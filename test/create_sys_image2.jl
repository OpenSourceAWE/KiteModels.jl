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
    precompile_execution_file=joinpath("test", "test_for_precompile.jl")
)
