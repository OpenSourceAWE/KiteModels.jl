# SPDX-FileCopyrightText: 2026 Uwe Fechner
# SPDX-License-Identifier: MIT

# Tests of the initial heading (set.headings[1]) and steering (set.steerings[1]) of KPS3 and KPS4.
using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using LinearAlgebra, Test
using KiteModels, KitePodModels

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

function new_model(Model; heading=0.0, steering=0.0, azimuth=0.0)
    set = deepcopy(load_settings("system.yaml"))
    set.heading  = heading
    set.steering = steering
    set.azimuth  = azimuth
    Model(KCU(set))
end

spring_lengths(s::KPS4) = [norm(s.pos[sp.p2] - s.pos[sp.p1]) for sp in s.springs]

@testset verbose=true "initial heading and steering" begin
    @testset "KPS4, heading $(heading)°, azimuth $(azimuth)°" for heading in (30.0, -60.0, 179.0),
                                                                azimuth in (0.0, 20.0)
        ref = new_model(KPS4; azimuth)
        @test !isnothing(init!(ref))
        s = new_model(KPS4; heading, azimuth)
        @test !isnothing(init!(s))
        @test calc_heading(s) ≈ mod2pi(deg2rad(heading)) atol=1e-8
        # the kite is turned about the line from the KCU to the kite: the position of the kite,
        # the tether and the lengths of all springs stay the same
        @test pos_kite(s) ≈ pos_kite(ref) atol=1e-9
        @test rad2deg(calc_azimuth(s)) ≈ azimuth atol=1e-6
        @test rad2deg(calc_elevation(s)) ≈ s.set.elevation atol=1e-6
        @test s.pos[1:s.set.segments+1] ≈ ref.pos[1:s.set.segments+1]
        @test spring_lengths(s) ≈ spring_lengths(ref) rtol=1e-12
    end

    @testset "KPS4, heading 90° and -90°: the kite flies sideways" begin
        azimuths = map((90.0, -90.0)) do heading
            s = new_model(KPS4; heading)
            integrator = init!(s)
            for _ in 1:20
                next_step!(s, integrator; set_speed=0.0, dt=0.05)
            end
            rad2deg(calc_azimuth(s))
        end
        @test azimuths[1] > 2.0
        @test azimuths[2] ≈ -azimuths[1] rtol=1e-3
    end

    @testset "KPS3 ignores the heading" begin
        s = new_model(KPS3; heading=30.0)
        @test !isnothing(init!(s))
        @test calc_heading(s) ≈ 0.0 atol=1e-6
    end

    @testset "$(nameof(Model)), steering 10%" for Model in (KPS3, KPS4)
        s = new_model(Model; steering=10.0)
        @test !isnothing(init!(s))
        @test s.kcu.steering ≈ 0.1
        @test s.kcu.set_steering ≈ 0.1
        @test s.kcu_steering ≈ 0.1
        # s.steering takes the steering offset and the influence of the depower into account
        @test 0.0 < s.steering < 0.1
        ref = new_model(Model)
        @test !isnothing(init!(ref))
        @test ref.steering ≈ (0 - ref.set.c0) / (1 + ref.set.k_ds * (ref.alpha_depower / deg2rad(ref.set.alpha_d_max)))
    end

    @testset "KPS4, steering 20%: the kite turns" begin
        s = new_model(KPS4; steering=20.0)
        integrator = init!(s)
        for _ in 1:40
            next_step!(s, integrator; set_speed=0.0, dt=0.05)
        end
        @test rad2deg(calc_heading(s)) > 20.0
    end
end
nothing
