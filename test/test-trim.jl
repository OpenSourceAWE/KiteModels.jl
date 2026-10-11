# SPDX-FileCopyrightText: 2026 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using Test
using KiteModels, KitePodModels

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

# Find the steady state, starting at the elevation el0 [°]. Returns the elevation [°] and the
# largest force residual (the residual of the accelerations of all particles).
function steady_state(Model, el0; trim, delta)
    set = deepcopy(load_settings("system.yaml"))
    set.elevation = el0
    s = Model(KCU(set))
    KiteModels.clear!(s)
    y0, yd0 = @test_logs min_level=Base.CoreLogging.Warn find_steady_state!(s; trim, delta)
    res = zeros(length(y0))
    residual!(res, yd0, y0, s, 0.0)
    if trim
        @test s.alpha_2 < s.set.alpha_cl[argmax(s.set.cl_list)]  # no deep stall
    end
    particles = Model == KPS4 ? s.set.segments + KiteModels.KITE_PARTICLES : s.set.segments
    n = 3 * particles
    rad2deg(calc_elevation(s)), maximum(abs, res[n+1:2n])
end

@testset "find_steady_state! with trim=true, $Model" for Model in (KPS3, KPS4)
    el_set = load_settings("system.yaml").elevation
    # without trim the elevation is prescribed, so the kite is not in equilibrium
    el, max_res = steady_state(Model, el_set; trim=false, delta=0.0)
    @test el ≈ el_set atol=1e-6
    @test max_res > 1.0
    # with trim all forces are balanced at the natural elevation of the kite
    el_trim, max_res = steady_state(Model, el_set; trim=true, delta=0.0)
    @test max_res < 1e-4
    @test abs(el_trim - el_set) > 0.5
    @test 60.0 < el_trim < 80.0
    # the trimmed elevation does not depend on the start value or on delta; from 30° the solver
    # finds a deep stall trim first, and for KPS4 it diverges from 55°, so both need restarts
    for el0 in (30.0, 55.0, 85.0)
        @test steady_state(Model, el0; trim=true, delta=0.0)[1] ≈ el_trim atol=1e-3
    end
    @test steady_state(Model, el_set; trim=true, delta=0.001)[1] ≈ el_trim atol=0.05
end
nothing
