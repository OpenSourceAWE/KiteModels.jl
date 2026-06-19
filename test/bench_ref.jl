# SPDX-FileCopyrightText: 2022 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end

const reference = 4.826620521958565e7 # AMD Ryzen 7840U, Julia 1.11, 1 thread

"""
    cpu_benchmark_scalar(target_time=1.0)

Performs scalar CPU-intensive operations without SIMD for approximately `target_time` seconds.
This function performs basic arithmetic operations on scalar values to measure CPU performance
without utilizing vector/SIMD instructions.

Returns the number of operations performed and the actual elapsed time.
"""
function cpu_benchmark_scalar(target_time=1.0)
    # First, do a short calibration run to estimate operations per second
    calibrate_ops = 1_000_000
    start_cal = Base.time()
    
    # Scalar operations that avoid SIMD optimization
    result = 0.0
    x = 1.23456789
    y = 9.87654321
    
    for _ in 1:calibrate_ops
        # Mix of operations to avoid compiler optimizations
        x = x * 1.000001 + sin(y * 0.001)
        y = y * 0.999999 + cos(x * 0.001)
        result += sqrt(abs(x + y))
    end
    
    cal_time = Base.time() - start_cal
    ops_per_second = calibrate_ops / cal_time
    
    # Estimate total operations needed for target time
    target_ops = Int(round(ops_per_second * target_time))
    
    # Main benchmark run
    start_time = Base.time()
    result = 0.0
    x = 1.23456789
    y = 9.87654321
    
    @inbounds for i in 1:target_ops
        # Scalar arithmetic operations that are hard to vectorize
        x = x * 1.000001 + sin(y * 0.001)
        y = y * 0.999999 + cos(x * 0.001)
        result += sqrt(abs(x + y))
        
        # Additional scalar operations to increase workload
        if i % 1000 == 0
            x = x / (1.0 + 1e-10)  # Prevent overflow
            y = y / (1.0 + 1e-10)
        end
    end
    
    elapsed_time = Base.time() - start_time
    
    # Force the compiler to use the result (prevent dead code elimination)
    println("Benchmark result (ignore): $(result)")

    return target_ops / elapsed_time
end

"""
    rel_cpu_performance()

A simple CPU benchmark that performs scalar operations for approximately 1 second.
This is a standalone function that doesn't depend on any external packages.
"""
function rel_cpu_performance()
    ops = cpu_benchmark_scalar(1.0)
    println("CPU Performance: $(round(ops/1e6, digits=1)) million operations per second")
    return ops/reference
end