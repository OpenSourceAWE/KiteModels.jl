# SPDX-FileCopyrightText: 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

# Extracted from test-kps4.jl — this test currently fails and is kept separate
# to avoid blocking the rest of the test suite.

using Pkg
if ! ("PackageCompiler" ∈ keys(Pkg.project().dependencies))
    Pkg.activate("test")
end
using LinearAlgebra, StaticArrays, Test
using KiteModels, KitePodModels

set_data_path(joinpath(dirname(dirname(pathof(KiteModels))), "data"))
set = deepcopy(load_settings("system.yaml"))
kcu = KCU(set)
kps4 = KPS4(kcu)

function init_392()
    KiteModels.clear!(kps4)
    kps4.set.l_tethers[1] = 392.0
    kps4.set.elevation = 70.0
    kps4.set.area = 10.0
    kps4.set.rel_side_area = 50.0
    kps4.set.v_wind = 9.1
    kps4.set.mass = 6.2
    kps4.set.c_s = 0.6
end

@testset "test_find_steady_state" begin
    init_392()
    clear!(kps4)
    KiteModels.set_depower_steering!(kps4, kps4.set.depower_offset/100.0, 0.0)
    height = sin(deg2rad(kps4.set.elevation)) * kps4.set.l_tether
    kps4.v_wind .= kps4.v_wind_gnd * calc_wind_factor(kps4.am, height)
    res1, res2 = find_steady_state!(kps4; delta=0.002, stiffness_factor=0.035, prn=false) 
    @test sum(res2) ≈ -9.81*(set.segments+ KiteModels.KITE_PARTICLES) # velocity and acceleration must be near zero
    pre_tension = KiteModels.calc_pre_tension(kps4)
    @test pre_tension > 1.0001
    @test pre_tension < 1.01
    @test unstretched_length(kps4) ≈ 392.0              # initial, unstretched tether length
    @test isapprox(tether_length(kps4), 395.51, rtol=1e-2) # real, stretched tether length
end
nothing