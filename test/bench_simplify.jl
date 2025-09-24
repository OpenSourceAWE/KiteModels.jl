# Copyright (c) 2024, 2025 Bart van de Lint, Uwe Fechner
# SPDX-License-Identifier: MIT

SIMPLE = false
T_REF = 48.0 # AMD Ryzen 7840U, Julia 1.11, no sys image [s]
             # 37s with sys image

using Pkg
if ! ("Test" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using KiteModels, LinearAlgebra, Statistics, Test, Distributed
include("bench_ref.jl")

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
    # Create a temporary script file for the benchmark
    benchmark_script = """
    using Pkg
    if ! ("Test" ∈ keys(Pkg.project().dependencies))
        using TestEnv; TestEnv.activate()
    end
    using KiteModels, LinearAlgebra, Statistics
    include("test/bench_ref.jl")
    
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
    @info "Simplify took \$time_ seconds"
    rel_performance = (T_REF / rel_cpu_performance())/time_
    
    # Write results to file for parent process to read
    open("benchmark_results.tmp", "w") do f
        println(f, time_)
        println(f, rel_performance)
    end
    """
    
    # Write the script to a temporary file
    temp_script = "temp_benchmark.jl"
    open(temp_script, "w") do f
        write(f, benchmark_script)
    end
    
    try
        # Run the benchmark in a separate Julia process
        result = run(`julia --project=. $temp_script`)
        
        if result.exitcode == 0
            # Read results from temporary file
            if isfile("benchmark_results.tmp")
                lines = readlines("benchmark_results.tmp")
                time_ = parse(Float64, lines[1])
                rel_performance = parse(Float64, lines[2])
                
                @info "Simplify took $time_ seconds"
                @info "Relative performance: $rel_performance"
                @test rel_performance > 0.8
                
                # Clean up temporary files
                rm("benchmark_results.tmp", force=true)
                rm(temp_script, force=true)
                
                return time_, rel_performance
            else
                error("Benchmark results file not found")
            end
        else
            error("Benchmark process failed with exit code $(result.exitcode)")
        end
    catch e
        # Clean up temporary files in case of error
        rm("benchmark_results.tmp", force=true)
        rm(temp_script, force=true)
        rethrow(e)
    end
end

# Run the benchmark in a separate process
time_, rel_performance = run_benchmark_subprocess()

# Note: sys object is not available when running in separate process
# If you need sys, you would need to serialize it or run parts in the main process
nothing

# Desktop, AMD Ryzen 9 7950X, Julia 1.11:
# - first  run 34.5 seconds
# - second run 21.1 seconds

# Laptop, AMD Ryzen 7 7840U, Julia 1.11:
# - first  run 35.0 seconds
# - second run 24.0 seconds


