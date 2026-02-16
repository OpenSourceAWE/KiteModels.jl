# SPDX-FileCopyrightText: 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

# MWE: NLsolve trust-region autoscale NaN bug
#
# Bug location:
#   NLsolve/src/solvers/trust_region.jl, initial autoscale block (~line 150):
#
#       for j = 1:nn
#           cache.d[j] = norm(view(jacobian(df), :, j))
#           if cache.d[j] == zero(cache.d[j])    # ← BUG: only catches exact 0.0
#               cache.d[j] = one(cache.d[j])
#           end
#       end
#
#   Later in the dogleg step (~line 78):
#
#       g .= g ./ d.^2                            # ← NaN when d[j]^2 underflows to 0.0
#
# Root cause:
#   When a Jacobian column has very tiny entries (e.g. ~1e-305), the column norm
#   d[j] is nonzero, so the guard `d[j] == 0` does NOT trigger. But d[j]^2
#   underflows to 0.0, and the subsequent division g ./ d.^2 produces NaN.
#
#   Any value below sqrt(floatmin(Float64)) ≈ 1.49e-154 will have its square
#   underflow to a subnormal or zero. The current guard only catches exact 0.0.
#
# Context:
#   NLSolversBase 7.10.0 switched from direct FiniteDiff to DifferentiationInterface.
#   For certain real-world systems the DI backend produces tiny near-zero values
#   in Jacobian columns that should be zero, instead of exact zeros. This triggers
#   the faulty guard above.
#
# Suggested fix:
#   Replace `d[j] == zero(d[j])` with a check that also catches values whose
#   square would underflow, e.g.:
#       if d[j] <= sqrt(floatmin(eltype(cache.d)))
#           cache.d[j] = one(cache.d[j])
#       end
#
# Tested with NLsolve 4.5.1, but the bug is in all versions with this autoscale guard.

using NLsolve, LinearAlgebra

# ----- Step 1: Show the arithmetic issue -----
println("=== Floating-point underflow in d.^2 ===")
println("floatmin(Float64) = ", floatmin(Float64), "  (smallest normal Float64)")
println("sqrt(floatmin)    = ", sqrt(floatmin(Float64)))
println()
for v in [1e-100, 1e-154, 1e-155, 1e-200, 1e-305]
    sq = v^2
    println("  d = $v  →  d^2 = $sq", sq == 0.0 ? "  ← UNDERFLOWS TO ZERO" : "")
end
println()

# ----- Step 2: Trigger NaN in NLsolve -----
# A simple 2-equation system. We provide an explicit Jacobian where column 2
# has tiny entries, simulating what DifferentiationInterface produces for
# near-zero partial derivatives in real applications.

function f!(F, x)
    F[1] = x[1] - 1.0
    F[2] = x[1]^2 - 4.0
    return F
end

function j!(J, x)
    J[1, 1] = 1.0
    J[2, 1] = 2.0 * x[1]
    # Column 2: tiny values that are NOT exactly zero.
    # These are above floatmin (so they are normal Float64), but their square
    # underflows to 0.0, causing division by zero in the dogleg step.
    J[1, 2] = 1e-200
    J[2, 2] = 1e-200
    return J
end

x0 = [0.5, 0.5]
println("=== NLsolve trust_region with autoscale=true ===")
println("Starting point: ", x0)

# Verify the Jacobian column norm issue
d2 = norm([1e-200, 1e-200])
println("Column 2 norm:    d = ", d2)
println("  d == 0.0:       ", d2 == 0.0, "  (guard does NOT trigger)")
println("  d^2:            ", d2^2, "  (underflows to zero!)")
println("  1.0 / d^2:      ", 1.0 / d2^2, "  → NaN propagation")
println()

result = nlsolve(f!, j!, x0, method=:trust_region, autoscale=true)
println("Converged: ", converged(result))
println("Solution:  ", result.zero)
println("NaN in solution: ", any(isnan, result.zero))
println()

# ----- Step 3: Show exact zeros work (guard triggers correctly) -----
println("=== Same system, but with EXACT zeros in Jacobian column 2: ===")
function j_zero!(J, x)
    J[1, 1] = 1.0;  J[2, 1] = 2.0 * x[1]
    J[1, 2] = 0.0;  J[2, 2] = 0.0          # exact zeros → d[j]==0 guard triggers
    return J
end
result2 = nlsolve(f!, j_zero!, x0, method=:trust_region, autoscale=true)
println("Converged: ", converged(result2))
println("Solution:  ", result2.zero)
println("NaN: ", any(isnan, result2.zero))
println()
println("The ONLY difference is 1e-200 vs 0.0 in Jacobian column 2.")
println("With 1e-200: d[j] ≠ 0 → guard skipped → d[j]^2 underflows → NaN")
println("With 0.0:    d[j] == 0 → guard triggers → d[j] = 1 → OK")
