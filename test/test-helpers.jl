# SPDX-FileCopyrightText: 2022, 2024, 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end

using Test
using KiteModels

function run_helper_tests()
    @testset "Testing helper functions..." begin
        original_dir = pwd()
        tmpdirs = []  # Collect temp directories to clean up at the end
        
        try
            # Test 1: copy_examples
            tmpdir1 = mktempdir()
            push!(tmpdirs, tmpdir1)
            cd(tmpdir1)
            KiteModels.copy_examples()
            @test isfile(joinpath(tmpdir1, "examples", "bench.jl"))
            @test isfile(joinpath(tmpdir1, "examples", "compare_kps3_kps4.jl"))
            @test isfile(joinpath(tmpdir1, "examples", "menu.jl"))
            @test isfile(joinpath(tmpdir1, "examples", "reel_out_1p.jl"))
            @test isfile(joinpath(tmpdir1, "examples", "reel_out_4p.jl"))
            @test isfile(joinpath(tmpdir1, "examples", "reel_out_4p_torque_control.jl"))
            @test isfile(joinpath(tmpdir1, "examples", "simulate_simple.jl"))
            @test isfile(joinpath(tmpdir1, "examples", "simulate_steering.jl"))
            cd(original_dir)
            
            # Test 2: install_examples
            # Reset DATA_PATH to default so it doesn't reference a deleted temp directory
            KiteUtils.set_data_path()
            tmpdir2 = mktempdir()
            push!(tmpdirs, tmpdir2)
            cd(tmpdir2)
            KiteModels.install_examples(false)
            @test isfile(joinpath(tmpdir2, "examples", "bench.jl"))
            @test isfile(joinpath(tmpdir2, "examples", "compare_kps3_kps4.jl"))
            @test isfile(joinpath(tmpdir2, "examples", "menu.jl"))
            @test isfile(joinpath(tmpdir2, "examples", "reel_out_1p.jl"))
            @test isfile(joinpath(tmpdir2, "examples", "reel_out_4p.jl"))
            @test isfile(joinpath(tmpdir2, "examples", "reel_out_4p_torque_control.jl"))
            @test isfile(joinpath(tmpdir2, "examples", "simulate_simple.jl"))
            @test isfile(joinpath(tmpdir2, "examples", "simulate_steering.jl"))
            cd(original_dir)

            @test ! ("TestEnv" ∈ keys(Pkg.project().dependencies))
            @test ! ("Revise" ∈ keys(Pkg.project().dependencies))
            @test ! ("Plots" ∈ keys(Pkg.project().dependencies))
        finally
            # Always restore original directory and reset DATA_PATH
            cd(original_dir)
            KiteUtils.set_data_path()
            # Clean up all temp directories
            if ! Sys.iswindows()
                for tmpdir in tmpdirs
                    isdir(tmpdir) && rm(tmpdir, recursive=true)
                end
            end
        end
    end
end

# Run the tests
run_helper_tests()

nothing
