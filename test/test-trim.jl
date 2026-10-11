# SPDX-FileCopyrightText: 2026 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using LinearAlgebra, Test
using KiteModels, KitePodModels

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

# Find the steady state, starting at the elevation el0 [°]. Returns the elevation [°] and the
# largest force residual (the residual of the accelerations of all particles).
function steady_state(Model, el0; trim)
    set = deepcopy(load_settings("system.yaml"))
    set.elevation = el0
    s = Model(KCU(set))
    KiteModels.clear!(s)
    y0, yd0 = @test_logs min_level=Base.CoreLogging.Warn find_steady_state!(s; trim)
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
    el, max_res = steady_state(Model, el_set; trim=false)
    @test el ≈ el_set atol=1e-6
    @test max_res > 1.0
    # with trim all forces are balanced at the natural elevation of the kite
    el_trim, max_res = steady_state(Model, el_set; trim=true)
    @test max_res < 1e-4
    @test abs(el_trim - el_set) > 0.5
    @test 50.0 < el_trim < 80.0
    # the trimmed elevation does not depend on the start value; from 30° the solver
    # finds a deep stall trim first, and for KPS4 it diverges from 55°, so both need restarts
    for el0 in (30.0, 55.0, 85.0)
        @test steady_state(Model, el0; trim=true)[1] ≈ el_trim atol=1e-3
    end
end

@testset "init! with trim=true, $Model" for Model in (KPS3, KPS4)
    # at full stiffness the trimmed initial state is an equilibrium of the simulation,
    # so the elevation stays constant
    s = Model(KCU(deepcopy(load_settings("system.yaml"))))
    integrator = @test_logs min_level=Base.CoreLogging.Warn init!(s; trim=true)
    el0 = calc_elevation(s)
    @test abs(rad2deg(el0) - s.set.elevation) > 0.5
    dt = 1/s.set.sample_freq
    max_dev = 0.0
    for _ in 1:round(Int, 10/dt)
        next_step!(s, integrator; set_speed=0.0, dt)
        max_dev = max(max_dev, abs(rad2deg(calc_elevation(s) - el0)))
    end
    @test max_dev < 0.2
end

# wind profiles of AtmosphericModels.jl, fitted to the near-shore location Maasvlakte, NL and
# to the inland location Cabauw, NL
const PROFILES = Dict("Maasvlakte" => (profile_law=3, alpha=0.08163, z0=0.0002),
                      "Cabauw"     => (profile_law=1, alpha=0.234,   z0=0.0002))

function profile_settings(profile, v_wind)
    set = deepcopy(load_settings("system.yaml"))
    p = PROFILES[profile]
    set.profile_law, set.alpha, set.z0, set.v_wind = p.profile_law, p.alpha, p.z0, v_wind
    set
end

@testset "trim for wind profile $profile, $Model" for Model in (KPS3, KPS4), profile in ("Maasvlakte", "Cabauw")
    elevations = Float64[]
    for v_wind in (4.0, 7.0, 10.0)
        # at 4 m/s, Maasvlakte has only 5.2 m/s at the height of the kite: the lift is too small
        # to carry the kite, so no trim with attached flow exists
        profile == "Maasvlakte" && v_wind == 4.0 && continue
        s = Model(KCU(profile_settings(profile, v_wind)))
        integrator = @test_logs min_level=Base.CoreLogging.Warn init!(s; trim=true)
        el0 = calc_elevation(s)
        push!(elevations, el0)
        # the kite is balanced for the wind of the profile at its height, with attached flow
        v_profile = v_wind * KiteModels.calc_wind_factor(s.am, calc_height(s))
        @test norm(s.v_wind) ≈ v_profile rtol=1e-6
        @test s.alpha_2 < s.set.alpha_cl[argmax(s.set.cl_list)]
        # the trimmed state is an equilibrium of the simulation, so the elevation stays constant
        dt = 1/s.set.sample_freq
        max_dev = 0.0
        for _ in 1:round(Int, 10/dt)
            next_step!(s, integrator; set_speed=0.0, dt)
            max_dev = max(max_dev, abs(rad2deg(calc_elevation(s) - el0)))
        end
        @test max_dev < 0.01
    end
    # the kite flies higher at higher wind speeds
    @test issorted(elevations)
end

@testset "deprecated keyword delta, $Model" for Model in (KPS3, KPS4)
    # delta is still accepted, but ignored: the steady state is the same as without it
    s = Model(KCU(deepcopy(load_settings("system.yaml"))))
    KiteModels.clear!(s)
    y_ref, _ = find_steady_state!(s)
    s = Model(KCU(deepcopy(load_settings("system.yaml"))))
    KiteModels.clear!(s)
    y, _ = find_steady_state!(s; delta=0.005)
    @test y ≈ y_ref
    @test !isnothing(init!(Model(KCU(deepcopy(load_settings("system.yaml")))); delta=0.005))
end

@testset "deprecated keyword stiffness_factor, $Model" for Model in (KPS3, KPS4)
    # stiffness_factor is still accepted, but ignored: the steady state is solved for the full
    # tether stiffness
    s = Model(KCU(deepcopy(load_settings("system.yaml"))))
    KiteModels.clear!(s)
    y_ref, _ = find_steady_state!(s)
    s = Model(KCU(deepcopy(load_settings("system.yaml"))))
    KiteModels.clear!(s)
    y, _ = find_steady_state!(s; stiffness_factor=0.1)
    @test y ≈ y_ref
    @test s.stiffness_factor == 1.0
    s = Model(KCU(deepcopy(load_settings("system.yaml"))))
    @test !isnothing(init!(s; stiffness_factor=0.1))
    @test s.stiffness_factor == 1.0
end

@testset "deep stall at a low prescribed elevation, $Model" for Model in (KPS3, KPS4)
    # 40° is far below the natural elevation: without trim, the kite can only be balanced in deep
    # stall, which is reported; with trim, the natural elevation is found
    set = deepcopy(load_settings("system.yaml"))
    set.elevation = 40.0
    s = Model(KCU(set))
    KiteModels.clear!(s)
    @test_logs (:warn, r"deep stall at the prescribed elevation") find_steady_state!(s)
    s = Model(KCU(deepcopy(set)))
    KiteModels.clear!(s)
    @test_logs min_level=Base.CoreLogging.Warn find_steady_state!(s; trim=true)
    @test s.alpha_2 < s.set.alpha_cl[argmax(s.set.cl_list)]
end
nothing
