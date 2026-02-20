# SPDX-FileCopyrightText: 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if ! ("Test" ∈ keys(Pkg.project().dependencies))
    Pkg.activate("test")
end
using LinearAlgebra, StaticArrays, Test
using KiteModels, KitePodModels

set_data_path(joinpath(dirname(dirname(pathof(KiteModels))), "data"))

function init_150(kps4)
    kps4.set.l_tethers[1] = 150.0
    kps4.set.elevation = 70.0
    kps4.set.area = 10.18
    kps4.set.rel_side_area = 30.6
    kps4.set.v_wind = 9.1
    kps4.set.mass = 6.21
    kps4.set.c_s = 0.6
    kps4.set.axial_damping = 473.0
    kps4.set.axial_stiffness = 614600.0
    kps4.set.width = 4.9622
    KiteModels.clear!(kps4)
end

function simulate(kps4, integrator, steps)
    iter = 0
    for i in 1:steps
        next_step!(kps4, integrator; set_speed=0)
        iter += kps4.iter
    end
    iter / steps
end

@testset "test_simulate        " begin
    local kps4
    kps4 = KPS4(KCU(deepcopy(load_settings("system.yaml"))))
    kps4.set.kcu_diameter = 0.0
    init_150(kps4)  
    kps4.set.elevation = 60.0
    kps4.set.alpha_zero = 0.0
    STEPS = 500
    kps4.set.depower = 23.6
    kps4.set.solver = "IDA"
    kps4.set.profile_law = Int(EXPLOG)
    kps4.set.alpha = 0.08163
    integrator = KiteModels.init!(kps4; stiffness_factor=0.4, delta=0.0015, prn=false)
    simulate(kps4, integrator, 100)
    av_steps = simulate(kps4, integrator, STEPS-100)
    if Sys.isapple()
        result = isapprox(av_steps, 300, rtol=0.6)
        if !result
            println("isapple $av_steps")
        end
        println("isapple $av_steps")
        @test result
    else
        result = isapprox(av_steps, 300, rtol=0.6)
        if !result
            println("not apple $av_steps")
        end
        @test result
    end

    lift, drag = KiteModels.lift_drag(kps4)
    # println(lift, " ", drag) # 703.7699568972286 161.44746368100536
    @test isapprox(lift, 520.5027844652874, rtol=0.05)
    sys_state = SysState(kps4)
    update_sys_state!(sys_state, kps4)
    # TODO Add testcase with varying reelout speed
end
nothing
