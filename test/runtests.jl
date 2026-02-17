# SPDX-FileCopyrightText: 2022, 2024, 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if ! ("Test" ∈ keys(Pkg.project().dependencies))
    Pkg.activate("test")
end

using KiteModels, KiteUtils
using Test

# Check if the compilation options allow maximum performance.
const build_is_production_build_env_name = "BUILD_IS_PRODUCTION_BUILD"
const build_is_production_build = let v = get(ENV, build_is_production_build_env_name, "true")
    if v ∉ ("false", "true")
        error("unknown value for environment variable $build_is_production_build_env_name: $v")
    end
    if v == "true"
        true
    else
        false
    end
end::Bool

cd("..")
KiteUtils.set_data_path("") 
@testset verbose = true "Testing KiteModels..." begin
    include("test-orientation.jl")
    println("--> 1")
    include("test-kps3.jl")
    println("--> 2")
    include("test-kps4.jl")
    println("--> 3")
    if build_is_production_build
        include("bench3.jl")
        include("bench4.jl")
    end
    include("test-helpers.jl")
    println("--> 4")
    include("test-inertia-calculation.jl")
    println("--> 5")
    include("test-interface.jl")
    println("--> 6")
    include("aqua.jl")
end