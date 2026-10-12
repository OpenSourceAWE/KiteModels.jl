# Copyright (c) 2024 Uwe Fechner and Bart van de Lint
# SPDX-License-Identifier: MIT

"""
    make_jac(f!, n_vars)

Create an explicit Jacobian function using central finite differences.

Entries whose squares would underflow are set to zero, so that the column norms used for
the scaling in `nl_solve` stay finite.
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
        # Flush near-zero entries whose squares would underflow, preventing NaN in the scaling
        @. J = ifelse(abs(J) < threshold, zero(SimFloat), J)
    end
    return jac!
end

# Result of nl_solve
struct NLResult
    zero::Vector{SimFloat}     # the solution
    f_converged::Bool          # true if the maximum norm of the residual is at most ftol
    x_converged::Bool          # true if the solver stalled
    residual_norm::SimFloat    # maximum norm of the residual
    iterations::Int
end

# Number of solves with updated scaling in nl_solve
const NL_SCALING_RESTARTS = 3

"""
    nl_solve(f!, jac!, x0; ftol, iterations, autoscale=true)

Solve `f!(F, x) = 0` with the trust region solver of NonlinearSolve, starting at `x0`, with the
Jacobian `jac!(J, x)`. Returns an `NLResult`.

If `autoscale` is `true`, the unknowns are scaled by the column norms of the Jacobian, like the
autoscaling of NLsolve. NLsolve updates the scaling in every iteration; here it is updated when
the solver is restarted from its last iterate, up to `NL_SCALING_RESTARTS` times.
"""
function nl_solve(f!, jac!, x0; ftol, iterations, autoscale=true)
    n = length(x0)
    # the solver works on z = x .* d
    d  = autoscale ? zeros(SimFloat, n) : ones(SimFloat, n)
    J0 = zeros(SimFloat, n, n)
    function update_scaling!(x)
        jac!(J0, x)
        for j in 1:n
            d[j] = max(d[j], norm(@view J0[:, j]))
            d[j] > 0 || (d[j] = one(SimFloat))
        end
    end
    xbuf = similar(x0)
    fz!(F, z, p) = (xbuf .= z ./ d; f!(F, xbuf))
    function jz!(J, z, p)
        xbuf .= z ./ d
        jac!(J, xbuf)
        J ./= d'
    end
    nf   = NonlinearFunction{true}(fz!; jac=jz!)
    alg  = TrustRegion(; radius_update_scheme=RadiusUpdateSchemes.NLsolve)
    term = NonlinearSolve.AbsNormTerminationMode(Base.Fix1(maximum, abs))
    x = copy(x0)
    F = similar(x0)
    residual_norm, steps, stalled = Inf, 0, false
    for _ in 1:(autoscale ? NL_SCALING_RESTARTS : 1)
        autoscale && update_scaling!(x)
        prob = NonlinearProblem(nf, x .* d)
        sol  = NonlinearSolve.solve(prob, alg; abstol=ftol, maxiters=iterations, termination_condition=term)
        xn = sol.u ./ d
        all(isfinite, xn) || break
        f!(F, xn)
        rn = maximum(abs, F)
        steps += sol.stats.nsteps
        # restart only while the residual decreases
        rn < residual_norm || break
        x, residual_norm = xn, rn
        stalled = sol.retcode == NonlinearSolve.ReturnCode.Stalled
        residual_norm <= ftol && break
    end
    NLResult(x, residual_norm <= ftol, stalled, residual_norm, steps)
end
