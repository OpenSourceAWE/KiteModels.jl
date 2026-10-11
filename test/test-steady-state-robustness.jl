# SPDX-FileCopyrightText: 2026 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using Test
using KiteModels, KitePodModels

# Cases for which a single nlsolve attempt in find_steady_state! did not converge when the
# initial state was perturbed by delta, so that the retries without autoscaling and the
# continuation in the stiffness factor were needed. The elevations are high enough for a steady
# state with attached flow: below about 45°, the kite can only be balanced in deep stall.
# Fields: v_wind [m/s], elevation [°], tether length [m]

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

const ROBUSTNESS_CASES = [
    (12.0, 60.0, 392.0),
    (9.51, 65.0, 150.0),
    (9.51, 70.0, 150.0),
    (6.0,  65.0, 150.0),
    (12.0, 70.0, 392.0),
    (6.0,  70.0, 392.0),
]

function robustness_kps4(v_wind, elevation, l_tether)
    set = deepcopy(load_settings("system.yaml"))
    set.v_wind = v_wind
    set.elevation = elevation
    set.l_tethers[1] = l_tether
    set.l_tether = l_tether
    KPS4(KCU(set))
end

@testset "find_steady_state! robustness" begin
    for (v_wind, elevation, l_tether) in ROBUSTNESS_CASES
        @testset "v_wind=$v_wind, elevation=$elevation, l_tether=$l_tether" begin
            kps4 = robustness_kps4(v_wind, elevation, l_tether)
            # a warning is logged if no usable steady state was found
            y0, yd0 = @test_logs min_level=Base.CoreLogging.Warn find_steady_state!(kps4)
            @test all(isfinite, y0)
            @test all(isfinite, yd0)
            # the steady state is solved for the full tether stiffness
            @test kps4.stiffness_factor == 1.0
            @test unstretched_length(kps4) ≈ l_tether
            # in a steady state the tether must be under tension; a result where the solver
            # stalled far away from the solution has a (nearly) unstretched tether
            pre_tension = KiteModels.calc_pre_tension(kps4)
            @test 1.0002 < pre_tension < 1.01
            # the elevation of the kite is prescribed by the settings
            @test rad2deg(calc_elevation(kps4)) ≈ elevation atol=1e-6

            kps4 = robustness_kps4(v_wind, elevation, l_tether)
            integrator = @test_logs min_level=Base.CoreLogging.Warn KiteModels.init!(kps4)
            @test !isnothing(integrator)
        end
    end
end
nothing
