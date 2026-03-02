# Copyright (c) 2024 Uwe Fechner and Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    make_jac(f!, n_vars)

Create an explicit Jacobian function using central finite differences.

Works around NLSolversBase >= 7.10 producing subnormal Jacobian entries via
DifferentiationInterface, which cause NaN in NLsolve autoscale (subnormal
column norms squared underflow to zero).
"""
function make_jac(f!, n_vars)
    _F1 = zeros(SimFloat, n_vars)
    _F2 = zeros(SimFloat, n_vars)
    _xp = zeros(SimFloat, n_vars)
    _xm = zeros(SimFloat, n_vars)
    threshold = sqrt(floatmin(SimFloat))
    function jac!(J, x)
        h_factor = cbrt(eps(SimFloat))
        for j in 1:n_vars
            copyto!(_xp, x)
            copyto!(_xm, x)
            h = max(abs(x[j]), one(SimFloat)) * h_factor
            _xp[j] += h
            _xm[j] -= h
            f!(_F1, _xp)
            f!(_F2, _xm)
            @views J[:, j] .= (_F1 .- _F2) ./ (2h)
        end
        # Flush near-zero entries whose squares would underflow, preventing NaN in autoscale
        @. J = ifelse(abs(J) < threshold, zero(SimFloat), J)
    end
    return jac!
end
