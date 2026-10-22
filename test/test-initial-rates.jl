# SPDX-FileCopyrightText: 2026 Uwe Fechner
# SPDX-License-Identifier: MIT

# Tests of the initial elevation, azimuth and heading rates (set.elevation_rates[1],
# set.azimuth_rates[1] and set.heading_rates[1]) of KPS3 and KPS4.
using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using LinearAlgebra, Test
using KiteModels, KitePodModels

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

function new_model(Model; elevation_rate=0.0, azimuth_rate=0.0, heading_rate=0.0, azimuth=0.0,
                   heading=0.0)
    set = deepcopy(load_settings("system.yaml"))
    set.elevation_rate = elevation_rate
    set.azimuth_rate   = azimuth_rate
    set.heading_rate   = heading_rate
    set.azimuth        = azimuth
    set.heading        = heading
    Model(KCU(set))
end

# Rates of the elevation, azimuth and heading [°/s] of the initial state of the integrator,
# by finite differences: the positions are moved by the velocities of the state times dt.
function finite_difference_rates(s, integrator; dt=1e-5)
    u = copy(integrator.u)
    n = 3 * (length(s.pos) - 1)
    res = zeros(length(u))
    angles() = rad2deg.((calc_elevation(s), calc_azimuth(s), calc_heading(s)))
    KiteModels.residual!(res, zeros(length(u)), u, s, 0.0)
    a0 = angles()
    u[1:n] .+= dt .* u[n+1:2n]
    KiteModels.residual!(res, zeros(length(u)), u, s, 0.0)
    d = (angles() .- a0) ./ dt
    (d[1], d[2], mod(d[3] + 180/dt, 360/dt) - 180/dt)   # the heading wraps at 360°
end

@testset verbose=true "initial rates" begin
    @testset "KPS4, rates $((elevation_rate, azimuth_rate, heading_rate)) °/s, azimuth $(azimuth)°, heading $(heading)°" for
            (elevation_rate, azimuth_rate, heading_rate, azimuth, heading) in
            ((2.0, 0.0, 0.0, 0.0, 0.0), (0.0, 5.0, 0.0, 0.0, 0.0), (0.0, 0.0, 10.0, 0.0, 0.0),
             (-3.0, 4.0, -8.0, 20.0, 60.0), (0.0, 0.0, 15.0, -30.0, 170.0))
        ref = new_model(KPS4; azimuth, heading)
        @test !isnothing(init!(ref))
        s = new_model(KPS4; elevation_rate, azimuth_rate, heading_rate, azimuth, heading)
        integrator = init!(s)
        @test !isnothing(integrator)
        # the positions are not changed
        @test s.pos ≈ ref.pos
        rates = finite_difference_rates(s, integrator)
        @test rates[1] ≈ elevation_rate atol=1e-3
        @test rates[2] ≈ azimuth_rate   atol=1e-3
        @test rates[3] ≈ heading_rate   atol=1e-3
    end

    @testset "KPS3, rates $((elevation_rate, azimuth_rate)) °/s, azimuth $(azimuth)°" for
            (elevation_rate, azimuth_rate, azimuth) in ((2.0, 0.0, 0.0), (0.0, 5.0, 0.0), (-3.0, 4.0, 20.0))
        s = new_model(KPS3; elevation_rate, azimuth_rate, azimuth)
        integrator = init!(s)
        @test !isnothing(integrator)
        rates = finite_difference_rates(s, integrator)
        @test rates[1] ≈ elevation_rate atol=1e-3
        @test rates[2] ≈ azimuth_rate   atol=1e-3
    end

    @testset "KPS3 ignores the heading rate" begin
        ref = new_model(KPS3)
        s = new_model(KPS3; heading_rate=10.0)
        @test init!(ref).u == init!(s).u
    end

    @testset "KPS4, pitch rate" begin
        s = new_model(KPS4)
        init!(s)
        @test s.pitch_rate == 0.0
        # rotating the kite in its vertical plane changes its pitch with the elevation rate
        s = new_model(KPS4; elevation_rate=2.0)
        init!(s)
        @test rad2deg(s.pitch_rate) ≈ -2.0 atol=1e-3
    end

    @testset "KPS4, azimuth rate 10°/s, heading 90°: the kite keeps moving" begin
        s = new_model(KPS4; azimuth_rate=10.0, heading=90.0)
        integrator = init!(s)
        for _ in 1:20
            next_step!(s, integrator; set_speed=0.0, dt=0.05)
        end
        @test 8.0 < rad2deg(calc_azimuth(s)) < 12.0
    end
end
nothing
