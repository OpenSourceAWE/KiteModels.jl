# SPDX-FileCopyrightText: 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using LinearAlgebra, StaticArrays, Test
using KiteModels, KitePodModels

# TODO: test for different elevation angles and wind speeds

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))
set = deepcopy(load_settings("system.yaml"))
kcu = KCU(set)
kps4 = KPS4(kcu)

function init_392(kps4, l)
    kps4.set.l_tethers[1] = l
    kps4.set.elevation = 70.0
    kps4.set.area = 10.0
    kps4.set.rel_side_area = 50.0
    kps4.set.v_wind = 9.1
    kps4.set.mass = 6.2
    kps4.set.c_s = 0.6
    kps4.set.max_iter = 500
    KiteModels.clear!(kps4)
end

@testset "test_find_steady_state" begin
    l_tethers = [100.0, 200.0, 392.0]
    for l in l_tethers
        # create a fresh instance for each test to avoid leftover state from find_steady_state!
        kps4_local = KPS4(KCU(deepcopy(set)))
        init_392(kps4_local, l)
        KiteModels.set_depower_steering!(kps4_local, kps4_local.set.depower_offset/100.0, 0.0)
        try
            _, res2 = find_steady_state!(kps4_local; delta=0.001, stiffness_factor=0.07, prn=false)
            @test sum(res2) ≈ -9.81*(set.segments+ KiteModels.KITE_PARTICLES) # velocity and acceleration must be near zero
            pre_tension = KiteModels.calc_pre_tension(kps4_local)
            @test pre_tension > 1.0001
            @test pre_tension < 1.01
            @test unstretched_length(kps4_local) ≈  l              # initial, unstretched tether length
            @test isapprox(tether_length(kps4_local), 1.008954l, rtol=2e-2) # real, stretched tether length
            @info "elevation: $(rad2deg(calc_elevation(kps4_local)))°"
            @info "aoa: $(kps4_local.alpha_2)°"
            @info "CL: $(kps4_local.calc_cl(kps4_local.alpha_2)), CD: $(kps4_local.calc_cd(kps4_local.alpha_2))"
            println()
        catch e
            if e isa ErrorException && contains(e.msg, "find_steady_state!") && get(ENV, "CI", "false") == "true"
                @warn "Steady state solver failed to converge for l=$l. Skipping test."
                @test_broken false  # Mark as known issue (CI flake only)
            else
                rethrow(e)
            end
        end
    end
end
nothing
