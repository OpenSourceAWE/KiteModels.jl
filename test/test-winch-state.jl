# SPDX-FileCopyrightText: 2026 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using Test
using KiteModels, KitePodModels

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

# state of the winch and of the set speed after init!
winch_state(s) = (s.sync_speed, s.wm.brake, s.wm.last_set_speed)

@testset "init! resets the winch state, $(nameof(Model))" for Model in (KPS3, KPS4)
    s = Model(KCU(load_settings("system.yaml")))
    integrator = KiteModels.init!(s; stiffness_factor=0.04)
    @test !isnothing(integrator)
    initial = winch_state(s)
    @test initial[1] == s.set.v_reel_out
    # leave the winch with another set speed, the brake released
    for _ in 1:20
        next_step!(s, integrator; set_speed=0.5)
    end
    @test !s.wm.brake
    @test s.wm.last_set_speed != initial[3]
    # set the brake to a state inside the hysteresis band that differs from the initial one
    s.sync_speed = s.wm.v_min
    s.wm.brake = !initial[2]
    integrator = KiteModels.init!(s; stiffness_factor=0.04)
    @test !isnothing(integrator)
    @test winch_state(s) == initial
end
