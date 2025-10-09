# Copyright (c) 2024, 2025 Bart van de Lint, Uwe Fechner
# SPDX-License-Identifier: MIT

SIMPLE = false
T_REF = 48.0 # AMD Ryzen 7840U, Julia 1.11, no sys image [s]
             # 37s with sys image
if VERSION.minor==12
    T_REF /= 0.70 # Julia 1.12 is about 30% slower on AMD Ryzen 7 7840U
end
if Sys.iswindows()
    T_REF /= 0.75 # Windows is about 25% slower than Linux on same hardware
end
msg = String[]

# TestEnv was previously used to activate the test environment dynamically.
# All required test dependencies are now listed under [targets] test in Project.toml,
# so we can rely on the active project without pulling in TestEnv.
using KiteModels, LinearAlgebra, Statistics, Test
include("bench_ref.jl")

# Repository root for subprocess scripts
const REPO_ROOT = normpath(@__DIR__, "..")

# Simulation parameters
dt = 0.05
total_time = 10.0  # Longer simulation to see oscillations
vsm_interval = 3
steps = Int(round(total_time / dt))

# Steering parameters
steering_freq = 1/2  # Hz - full left-right cycle frequency
steering_magnitude = 10.0      # Magnitude of steering input [Nm]

# Function to run benchmark in separate Julia process
function run_benchmark_subprocess()
    # Use an isolated temporary directory for all intermediate files
    mktempdir() do tmpdir
        results_file = joinpath(tmpdir, "benchmark_results.tmp")
        temp_script  = joinpath(tmpdir, "temp_benchmark.jl")

        # Create benchmark script with absolute results path embedded
        benchmark_script = """
        const REPO_ROOT = $(repr(REPO_ROOT))
        const RESULTS_FILE = $(repr(results_file))
        cd(REPO_ROOT)  # ensure consistent base directory
        using Pkg
        using KiteModels, LinearAlgebra, Statistics
        include(joinpath(REPO_ROOT, "test", "bench_ref.jl"))

        SIMPLE = $SIMPLE
        T_REF = $T_REF

        # Initialize model
        set = load_settings("system_ram.yaml")
        set.segments = 3
        set_values = [-50, 0.0, 0.0]  # Set values of the torques of the three winches. [Nm]
        set.quasi_static = false
        set.physical_model = SIMPLE ? "simple_ram" : "ram"

        sam = SymbolicAWEModel(set)
        sam.set.abs_tol = 1e-2
        sam.set.rel_tol = 1e-2
        rm("data/model_1.11_ram_dynamic_3_seg.bin"; force=true)

        # Initialize at elevation
        set.l_tethers[2] += 0.2
        set.l_tethers[3] += 0.2
        time_ = init!(sam; remake=false, reload=true, bench=true)
        rel_performance = (T_REF / rel_cpu_performance())/time_

        # Write results to file for parent process to read
        open(RESULTS_FILE, "w") do f
            println(f, time_)
            println(f, rel_performance)
        end
        """

        # Write the script to the temporary directory
        open(temp_script, "w") do f
            write(f, benchmark_script)
        end

        success = false
        time_ = NaN
        relp = NaN
        msg = nothing
        try
            result = run(`julia --project -t 1 $temp_script`)
            if result.exitcode == 0 && isfile(results_file)
                lines = readlines(results_file)
                time_ = parse(Float64, lines[1])
                relp = parse(Float64, lines[2])
                success = true
            else
                msg = "Benchmark subprocess exit=$(result.exitcode) file_exists=$(isfile(results_file))"
            end
        catch e
            io = IOBuffer(); showerror(io, e); msg = String(take!(io))
        finally
            # temp dir and contents auto-removed after mktempdir do-block
        end
        return success, time_, relp, msg
    end
end

ok, time_, rel_performance, err_msg = run_benchmark_subprocess()
if ! isnan(rel_performance)
    push!(msg,  ("Rel performance of simplify:      $(round(rel_performance, digits=2))"))
else
    push!(msg,  ("Error in simplify benchmark:      $(err_msg)"))
end

@testset "Testing performance of simplify..." begin
    if ok
        push!(msg, ("Simplify took:                   $(round(time_, digits=3)) s"))
        @test rel_performance > 0.8
    else
        @error "Simplify benchmark failed" err_msg
        @test ok  # will fail in strict mode
    end
end

printstyled("\nBenchmark results for simplify:\n"; bold = true)
for i in eachindex(msg)
    println(msg[i])
end
println()

# Note: sys object is not available when running in separate process
# If you need sys, you would need to serialize it or run parts in the main process
nothing

# Desktop, AMD Ryzen 9 7950X, Julia 1.11:
# - first  run 34.5 seconds
# - second run 21.1 seconds

# Laptop, AMD Ryzen 7 7840U, Julia 1.11:
# - first  run 35.0 seconds
# - second run 24.0 seconds


