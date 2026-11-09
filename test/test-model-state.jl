# SPDX-FileCopyrightText: 2026 Bart van de Lint
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using KiteModels, KiteUtils, LinearAlgebra, Test
using KiteModels: init!

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

# point i of the particles of the state vector u, whose points start after offset
state_point(u, offset, i) = i == 1 ? zeros(3) : u[offset+3i-5:offset+3i-3]

# a model after five time steps from init!, with its integrator and settings
function stepped_model(Model)
    set = load_settings("system.yaml")
    s = Model(KCU(set))
    integrator = init!(s; prn=false)
    for _ in 1:5
        next_step!(s, integrator; set_speed=1, dt=1/set.sample_freq)
    end
    s, integrator, set
end

@testset "the getters read the accepted state of the integrator, $Model" for
        Model in (KPS3, KPS4)
    s, integrator, set = stepped_model(Model)
    u = integrator.u
    P = length(s.pos)
    i_kite = Model == KPS4 ? P - 2 : P
    pos = [state_point(u, 0, i) for i in 1:P]
    vel = [state_point(u, 3 * (P - 1), i) for i in 1:P]
    @test s.pos == pos
    @test s.vel == vel
    @test s.vel_kite == vel[i_kite]
    @test pos_kite(s) == pos[i_kite]
    @test calc_elevation(s) == KiteUtils.calc_elevation(pos[i_kite])
    @test tether_length(s) ≈ sum(norm(pos[i+1] - pos[i]) for i in 1:set.segments)
    @test unstretched_length(s) == u[end-1]
    @test reel_out_speed(s) == u[end]
end

@testset "find_steady_state! reads the state it solves for, not the last run, $Model" for
        Model in (KPS3, KPS4)
    s, integrator, set = stepped_model(Model)
    y0, _ = find_steady_state!(s)
    @test s.pos[2] == state_point(y0, 0, 2)
    next_step!(s, integrator; set_speed=0, dt=1/set.sample_freq)
    @test s.pos[2] == state_point(integrator.u, 0, 2)
end

@testset "a model with no integrator reads the state residual! evaluated last, $Model" for
        Model in (KPS3, KPS4)
    s = Model(KCU(load_settings("system.yaml")))
    y, yd = KiteModels.init(s)
    y[1] += 1.0
    KiteModels.residual!(zeros(length(y)), yd, y, s, 0.0)
    @test s.pos[2] == state_point(y, 0, 2)
end

@testset "clear! forgets the integrator of the last run, $Model" for Model in (KPS3, KPS4)
    s, _, _ = stepped_model(Model)
    KiteModels.clear!(s)
    @test isnothing(s.integrator)
end

@testset "the KPS3 steering correction uses the elevation of the state evaluated" begin
    set = load_settings("system.yaml")
    s = KPS3(KCU(set))
    y, yd = KiteModels.init(s)
    res = zeros(length(y))
    kite = 3 * (set.segments - 1)
    KiteModels.residual!(res, yd, y, s, 0.0)
    y[3:3:3*set.segments] .*= 0.5
    KiteModels.residual!(res, yd, y, s, 0.0)
    @test s.beta ≈ KiteUtils.calc_elevation(y[kite+1:kite+3])
end
