# SPDX-FileCopyrightText: 2026 Uwe Fechner
# SPDX-License-Identifier: MIT

# Tests of the initial azimuth (set.azimuths[1]) of the steady state of KPS3 and KPS4.
using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using LinearAlgebra, Test
using KiteModels, KitePodModels

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

function new_model(Model, azimuth)
    set = deepcopy(load_settings("system.yaml"))
    set.azimuth = azimuth
    Model(KCU(set))
end

# components of a horizontal vector in the vertical plane of the kite and normal to it, for a kite
# at the given azimuth, with the wind coming from upwind_dir
function plane_components(upwind_dir, azimuth)
    turnangle = upwind_dir - deg2rad(azimuth) + π/2
    in_plane(x, y) = cos(turnangle) * x - sin(turnangle) * y
    normal(x, y)   = sin(turnangle) * x + cos(turnangle) * y
    in_plane, normal
end

@testset verbose=true "initial azimuth" begin
    @testset "$(nameof(Model)), azimuth $(azimuth)°, upwind_dir $(round(rad2deg(upwind_dir)))°" for
            Model in (KPS3, KPS4), azimuth in (30.0, -45.0), upwind_dir in (-π/2, π/4)
        s = new_model(Model, azimuth)
        res1, res2 = @test_logs min_level=Base.CoreLogging.Warn find_steady_state!(s; upwind_dir)
        res = zeros(length(res1))
        KiteModels.residual!(res, res2, res1, s, 0.0)
        @test rad2deg(calc_azimuth(s)) ≈ azimuth atol=1e-6
        @test rad2deg(calc_elevation(s)) ≈ s.set.elevation atol=1e-6
        # the tether particles are balanced, also normal to the vertical plane of the kite
        in_plane, normal = plane_components(upwind_dir, azimuth)
        segments = s.set.segments
        n = Model == KPS3 ? 3segments : 3*(segments + KiteModels.KITE_PARTICLES)
        for i in 1:segments-1
            j = n + 3*(i-1)
            @test abs(in_plane(res[j+1], res[j+2])) < 1e-4
            @test abs(normal(res[j+1], res[j+2])) < 1e-4
        end
        # with the wind at an angle, the tether is pushed sideways: the particles in the middle
        # of the tether are not in the vertical plane of the kite
        mid = res1[3*(segments÷2-1)+1:3*(segments÷2)]
        @test sign(normal(mid[1], mid[2])) == -sign(azimuth)
    end

    @testset "$(nameof(Model)), init! at azimuth 30°" for Model in (KPS3, KPS4)
        s = new_model(Model, 30.0)
        @test !isnothing(init!(s))
        @test rad2deg(calc_azimuth(s)) ≈ 30.0 atol=1e-6
        s = new_model(Model, 30.0)
        @test !isnothing(init!(s; steady_state=false))
        @test rad2deg(calc_azimuth(s)) ≈ 30.0 atol=1e-6
    end

    @testset "KPS3, turbulent wind at the turned kite position" begin
        set = deepcopy(load_settings("system.yaml"))
        set.azimuth = 30.0
        set.use_turbulence = 0.1
        s = KPS3(KCU(set))
        upwind_dir = π/4
        KiteModels.init(s; upwind_dir)
        pos = s.pos[end]
        v_wind, _ = calc_turbulent_wind(s.am, pos, upwind_dir, s.t_0)
        @test s.v_wind ≈ v_wind
    end

    @testset "$(nameof(Model)), azimuth 0° is unchanged" for Model in (KPS3, KPS4)
        s = new_model(Model, 0.0)
        res1, _ = find_steady_state!(s)
        @test rad2deg(calc_azimuth(s)) ≈ 0.0 atol=1e-6
        @test length(res1) == (Model == KPS3 ? 6s.set.segments + 2 :
                               6*(s.set.segments + KiteModels.KITE_PARTICLES) + 2)
    end
end
nothing
