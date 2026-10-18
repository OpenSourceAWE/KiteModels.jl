# SPDX-FileCopyrightText: 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end

using LinearAlgebra, StaticArrays, Test
using KiteModels, KitePodModels

const SEGMENTS = load_settings("system.yaml").segments

@testset verbose = true "KPS3 tests...." begin

    function set_defaults(kps)
        kps.set.l_tethers[1] = 150.0
        kps.set.elevation = 60.0
        kps.set.area = 20.0
        kps.set.rel_side_area = 50.0
        kps.set.v_wind = 8.0
        kps.set.mass = 11.4
        kps.set.axial_damping = 2 * 473.0
        kps.set.alpha = 1.0/7
        kps.set.c_s = 0.6
        KiteModels.clear!(kps)
    end

    function init_392(kps)
        kps.set.l_tethers[1] = 392.0
        kps.set.elevation = 70.0
        kps.set.area = 10.0
        kps.set.rel_side_area = 50.0
        kps.set.v_wind = 9.1
        kps.set.mass = 6.2
        kps.set.c_s = 0.6
        kps.set.alpha = 1.0/7
        KiteModels.clear!(kps)
    end

    @testset "calc_rho             " begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        set_defaults(kps)
        @test isapprox(calc_rho(kps.am, 0.0), 1.225, atol = 1e-5)
        @test isapprox(calc_rho(kps.am, 100.0), 1.210756, atol = 1e-5)
    end

    @testset "calc_wind_factor     " begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        set_defaults(kps)
        @test isapprox(calc_wind_factor(kps.am, 6.0, Int(EXP)), 1.0, atol = 1e-5)
        @test isapprox(calc_wind_factor(kps.am, 10.0, Int(EXP)), 1.0757037, atol = 1e-5)
        @test isapprox(calc_wind_factor(kps.am, 100.0, Int(EXP)), 1.494685, atol = 1e-5)
    end

    @testset "calc_cl              " begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        @test isapprox(kps.calc_cl(-5.0), 0.150002588978, atol = 1e-4)
        @test isapprox(kps.calc_cl(0.0), 0.200085035326, atol = 1e-4)
        @test isapprox(kps.calc_cl(10.0), 0.574103590856, atol = 1e-4)
        @test isapprox(kps.calc_cl(20.0), 1.0, atol = 1e-4)
    end

    @testset "test_calc_drag       " begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        v_segment = KVec3(1.0, 2, 3)
        unit_vector = KVec3(2.0, 3.0, 4.0)
        rho = SimFloat(calc_rho(kps.am, 10.0))
        v_app_perp = KVec3(0, -3.0, -4.0)
        area = 20.0
        kps.v_wind_tether .= [0.1, 0.2, 0.3]
        v_app_norm =
            KiteModels.calc_drag(kps, v_segment, unit_vector, rho, v_app_perp, area)
        @test v_app_norm ≈ 3.3674916481
        @test kps.last_tether_drag ≈ [-38506.7140169, -57266.39520463, -76026.07639235]
        @test v_app_perp ≈ [35.1, 52.2, 69.3]
    end

    @testset "test_calc_aero_forces" begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        set_defaults(kps)
        kps.v_apparent .= KVec3(35.1, 52.2, 69.3)
        kps.v_wind .= kps.v_wind_gnd
        pos_kite = KVec3(30.0, 5.0, 100.0)
        v_kite = KVec3(3.0, 5.0, 2.0)
        rho = SimFloat(calc_rho(kps.am, 10.0))
        rel_steering = -0.1
        kps.beta = 0.1
        kps.psi = 0.2
        kps.param_cl = 0.2
        kps.param_cd = 1.0
        KiteModels.calc_aero_forces(kps, pos_kite, v_kite, rho, rel_steering)
        @test kps.v_apparent ≈ [5.0, -5, -2]                                # same with Python
        @test kps.kite_y ≈ [0.64101597, 0.73258967, -0.22893427]           # same with Python
        @test kps.cor_steering ≈ 0.0250173783309
        @test kps.steering_force ≈ [-15.88482337, -18.15408385, 5.6731512]
        @test kps.last_force ≈ [-555.24319976, 544.82004621, 80.49946362]    # same with Python
    end

    @testset "test_calc_res        " begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        set_defaults(kps)
        i = 2
        pos1 = KVec3(30.0, 5.0, 100.0)
        pos2 = KVec3(30.0+10, 5.0+11, 100.0+20)
        vel1 = KVec3(3.0, 5.0, 2.0)
        vel2 = KVec3(3.0+0.1, 5.0+0.2, 2.0+0.3)
        mass = 9.0
        veld = KVec3(0.1, 0.3, 0.4)
        result = KVec3(0, 0, 0)
        kps.axial_stiffness = 0.011
        kps.axial_damping = 0.01
        kps.last_tether_drag = KVec3(5.0, 6, 7)
        kps.last_force = KVec3(-1.0, -2, -3)
        kps.v_app_perp = KVec3(0.1, 0.22, 0.33)
        kps.v_wind_tether .= [0.1, 0.2, 0.3]
        kps.segment_length = 10.0
        KiteModels.calc_res(kps, pos1, pos2, vel1, vel2, mass, veld, result, i)
        @test result ≈ [-6.99178740e-03, 1.01297086e-01, 9.83843908e+00]
        i = SEGMENTS+1
        KiteModels.calc_res(kps, pos1, pos2, vel1, vel2, mass, veld, result, i)
        @test result ≈ [0.14263997, 0.41669208, 10.12449937]
    end

    @testset "test_calc_loop       " begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        set_defaults(kps)
        kps.last_tether_drag = KVec3(5.0, 6, 7)
        kps.last_force = KVec3(-1.0, -2, -3)
        kps.v_app_perp = KVec3(0.1, 0.22, 0.33)
        kps.v_wind_tether .= [0.1, 0.2, 0.3]
        kps.segment_length = 10.0
        kps.axial_stiffness = kps.set.axial_stiffness / kps.segment_length
        kps.axial_damping = kps.set.axial_damping / kps.segment_length
        pos = zeros(SVector{SEGMENTS+1,KVec3})
        for i = 1:(SEGMENTS+1)
            pos[i][3] = 5.0 * (i-1)
        end
        vel = zeros(SVector{SEGMENTS+1,KVec3})
        posd = zeros(SVector{SEGMENTS+1,KVec3})
        veld = zeros(SVector{SEGMENTS+1,KVec3})
        res1 = zeros(SVector{SEGMENTS+1,KVec3})
        res2 = zeros(SVector{SEGMENTS+1,KVec3})
        @test kps.axial_stiffness ≈ 61460.0
        @test kps.axial_damping ≈ 94.6
        KiteModels.loop(kps, pos, vel, posd, veld, res1, res2)
        @test sum(res1) ≈ [0.0, 0.0, 0.0]
        @test isapprox(res2[7], [-5.02874357e-02, -1.00574871e-01, -7.62063430e+02])
        @test isapprox(res2[6], [-2.37861451e-03, -4.75722903e-03, 9.81000000e+00])
        @test isapprox(res2[5], [-2.38000593e-03, -4.76001187e-03, 9.81000000e+00])
        @test isapprox(
            res2[2],
            [-2.38139816e-03, -4.76279631e-03, 9.81000000e+00],
            rtol = 1e-5,
        )
        @test isapprox(res2[1], [0.0, 0.0, 0.0], rtol = 1e-5)
    end

    @testset "test_calc_alpha      " begin
        v_app = KVec3(10, 2, 3)
        vec_z = normalize(KVec3(3, 2, 0))
        alpha = KiteModels.calc_alpha(v_app, vec_z)
        @test alpha ≈ -1.091003745821884
    end

    @testset "test_set_cl_cd       " begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        alpha = deg2rad(10.0)
        KiteModels.set_cl_cd!(kps, alpha)
        @test kps.param_cl ≈ 0.5740976324353215
        @test kps.param_cd ≈ 0.12533689264639403
    end

    @testset "test_calc_set_cl_cd  " begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        kps.kcu.depower = 0.236
        kps.kcu.set_depower = kps.kcu.depower
        KiteModels.set_depower_steering!(kps, get_depower(kps.kcu), get_steering(kps.kcu))
        v_app = KVec3(10, 2, 3)
        vec_c = KVec3(3, 2, 0)
        KiteModels.calc_set_cl_cd!(kps, vec_c, v_app)
        @test kps.param_cl ≈ -0.09238222805380593
        @test kps.param_cd ≈ 0.8117345278100984
    end

    @testset "test_clear           " begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        kps.t_0 = 10.0
        KiteModels.clear!(kps)
        @test kps.t_0 == 0.0
        @test kps.v_reel_out == 0.0
        @test kps.last_v_reel_out == 0.0
    end

    @testset "test_v_wind_vert     " begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        set_defaults(kps)
        height = 100.0

        # default is 0.0, reproducing today's behaviour exactly
        @test kps.v_wind_vert == 0.0
        KiteModels.set_v_wind_ground!(kps, height)
        @test kps.v_wind[3] == 0.0
        horiz_no_vert = copy(kps.v_wind[1:2])

        # a nonzero v_wind_vert is added to the vertical component, not overwriting it,
        # and leaves the horizontal components, v_wind_gnd and v_wind_tether untouched
        kps.v_wind_vert = 1.0
        KiteModels.set_v_wind_ground!(kps, height)
        @test kps.v_wind[3] ≈ 1.0
        @test kps.v_wind[1:2] ≈ horiz_no_vert
        @test kps.v_wind_gnd[3] == 0.0
        @test kps.v_wind_tether[3] == 0.0
        @test upwind_dir(kps) ≈ -pi/2

        # clear! resets it
        KiteModels.clear!(kps)
        @test kps.v_wind_vert == 0.0
    end

    @testset "test_v_wind_vert turb" begin
        # regression: with turbulence switched on, v_wind_vert must be added to the turbulent
        # vertical wind, not replace it (a plain assignment would silently delete it)
        set_ = deepcopy(se())
        set_.use_turbulence = 0.1
        local kps = KPS3(KCU(set_))
        height = 100.0

        KiteModels.set_v_wind_ground!(kps, height)
        v_wind_turb_only = copy(kps.v_wind)

        kps.v_wind_vert = 2.0
        KiteModels.set_v_wind_ground!(kps, height)
        @test kps.v_wind[3] ≈ v_wind_turb_only[3] + 2.0
        @test kps.v_wind[1:2] ≈ v_wind_turb_only[1:2]
    end

    # Inputs:
    # State vector state_y   = pos1, pos2, ..., posn, vel1, vel2, ..., veln
    # Derivative   der_yd    = vel1, vel2, ..., veln, acc1, acc2, ..., accn
    # Output:
    # Residual     res = res1, res2 = pos1,  ..., vel1, ...
    @testset "test_residual!       " begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        res1 = zeros(SVector{SEGMENTS,KVec3})
        res2 = deepcopy(res1)
        res = reduce(vcat, vcat(res1, res2))

        res = vcat(res, zeros(2))
        X = zeros(SimFloat, 2*kps.set.segments)
        y0, yd0 = KiteModels.init(kps, X)
        # println(y0)
        # println(yd0)
        p = kps
        clear!(kps)
        residual!(res, yd0, y0, p)
        res1 = res[1:(3*SEGMENTS)]
        res2 = res[(3*SEGMENTS+1):end]
        @test res1 == zeros(3*(SEGMENTS))
        # TODO: add test for res2
        # println(res2)
    end

    @testset "test_set_depower_steering" begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        depower = 0.25
        steering = 0.1
        KiteModels.set_depower_steering!(kps, depower, steering)
        @test kps.depower == depower
        @test kps.steering ≈ 0.09034121603653548
    end

    @testset "test_init            " begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        set_defaults(kps)
        clear!(kps)
        my_state = deepcopy(kps)
        y0, yd0 = KiteModels.init(my_state, zeros(SimFloat, 2*SEGMENTS), delta = 1e-6)
        @test length(y0) == (SEGMENTS) * 6 + 2
        @test length(yd0) == (SEGMENTS) * 6 + 2
        @test sum(y0[1:(end-2)]) ≈ 717.163369868302
        @test sum(yd0) ≈ 3.6e-5
        @test isapprox(my_state.param_cl, 0.574103590856, atol = 1e-4)
        @test isapprox(my_state.param_cd, 0.125342896308, atol = 1e-4)
    end

    function test_initial_condition(kps, params::Vector)
        res1_ = zeros(SVector{SEGMENTS+1,KiteModels.KVec3})
        res2_ = deepcopy(res1_)
        res3 = vcat(reduce(vcat, vcat(res1_, res2_)), zeros(2))
        my_state = kps
        y0, yd0 = KiteModels.init(my_state, params)
        residual!(res3, yd0, y0, kps)
        return norm(res3) # z component of force on all particles but the first
    end

    @testset "test_initial_residual" begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        # global res, x, z
        init_392(kps)
        initial_x = [
            -1.52505,
            -3.67761,
            -5.51761,
            -6.08916,
            -4.41371,
            0.902124,
            0.366393,
            0.909132,
            1.27537,
            1.1538,
            0.300657,
            -1.51768,
        ]
        res = test_initial_condition(kps, initial_x)

        my_state = kps
        kps.set.l_tethers[1] = 392.0
        kps.set.elevation = 70.0
        kps.set.area = 10.0
        kps.set.v_wind = 9.1
        kps.set.mass = 6.2
        KiteModels.clear!(my_state)
        alpha = deg2rad(10.0)
        KiteModels.set_cl_cd!(kps, alpha)

        # println("state.param_cl: $(my_state.param_cl), state.param_cd: $(my_state.param_cd)")
        # println("res2: "); display(my_state.res2)
        # println("pos: "); display(my_state.pos)
        x = Float64[]
        z = Float64[]
        for i = 1:length(my_state.pos)
            push!(x, my_state.pos[i][1])
            push!(z, my_state.pos[i][3])
        end
        # println(norm(res))

        @test my_state.segment_length ≈ 65.33333333333333
        @test my_state.axial_stiffness ≈ 9407.142857142859
        @test my_state.axial_damping ≈ 14.479591836734695
        # @test isapprox(my_state.param_cl, 1.0641931441572074, atol=1e-4)
        # @test isapprox(my_state.param_cd, 0.22825898470541978, atol=1e-4)
        @test sum(my_state.res1) ≈ [0.0, 0.0, 0.0]
        @test my_state.res2[1] ≈ [0.0, 0.0, 0.0]

        # @test isapprox(my_state.res2[2], [8.83559075e+00, -4.72588546e-07, -5.10109289e+00], rtol=3e-2)
        # @test isapprox(my_state.res2[3], [8.81318565e+00, -4.68864292e-07, -5.08829453e+00], rtol=1e-3)
        # @test isapprox(my_state.res2[7], [1.49735632e+01,  2.71870215e-06,  4.51115984e+01], rtol=1e-3)
        # println("res2: "); display(my_state.res2)
        # println("lift force: $(norm(my_state.lift_force)) N")
    end

    @testset "test_getters" begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        init_392(kps)
        initial_x = [
            -1.52505,
            -3.67761,
            -5.51761,
            -6.08916,
            -4.41371,
            0.902124,
            0.366393,
            0.909132,
            1.27537,
            1.1538,
            0.300657,
            -1.51768,
        ]
        test_initial_condition(kps, initial_x)
        KiteModels.clear!(kps)
        KiteModels.set_cl_cd!(kps, deg2rad(10.0))
        x, y, z = kite_ref_frame(kps)
        @test all(x .≈ [-0.9070010101306292, 0.0, 0.4211284455151642])
        @test all(y .≈ [0.0, 1.0, 0.0])
        @test all(z .≈ [-0.4211284455151642, -0.0, -0.9070010101306292])
        @test all(
            KiteModels.orient_euler(kps) .≈ [0.0, 0.4346891114736793, -1.5707963267948966],
        )
        @test all(pos_kite(kps) .≈ [134.97402018366216, 0.0, 366.8418273480761])
        @test calc_elevation(kps) .≈ 1.2182337959242815 # 69.8 deg
        @test calc_azimuth(kps) ≈ 0
        @test mod2pi(calc_heading(kps) + π) ≈ π
        calc_course(kps) # the course for vel_kite=zero is undefined, so we cannot test it
    end

    @testset "test_find_steady_state" begin
        local kps = KPS3(KCU(load_settings("system.yaml")))
        init_392(kps)
        KiteModels.set_depower_steering!(kps, 0.25, 0.0)
        # the solver must converge without warning; the kite is placed at the elevation from the settings
        res1, res2 = @test_logs min_level=Base.CoreLogging.Warn find_steady_state!(kps; prn = false)
        @test rad2deg(calc_elevation(kps)) ≈ 70.0 atol=1e-6
        # the tether particles are in equilibrium (x and z components of their accelerations)
        res = zeros(length(res1))
        KiteModels.residual!(res, res2, res1, kps)
        segments = kps.set.segments
        for i in 1:segments-1
            j = 3*(i-1) + 3*segments
            @test abs(res[j+1]) < 1e-5
            @test abs(res[j+3]) < 1e-5
        end
        @test abs(res[3*(segments-1) + 3*segments + 1]) < 1e-5     # horizontal balance of the kite
        @test norm(res2) < 1e-5                            # velocity and acceleration must be near zero
        pre_tension = KiteModels.calc_pre_tension(kps)
        @test pre_tension > 1.0001
        @test pre_tension < 1.01
        @test unstretched_length(kps) ≈ 392.0                        # initial, unstretched tether length
        @test tether_length(kps) ≈ 392.2025410439266 rtol = 1e-5     # real, stretched tether length
        @test winch_force(kps) ≈ 306.73663152733974 rtol = 3e-2      # initial force at the winch [N]
        lift, drag = lift_drag(kps)
        @test lift ≈ 315.7822617990478 rtol=2e-2                # initial lift force of the kite [N]
        @test drag ≈ 71.44409941779216 rtol=2e-2                 # initial drag force of the kite [N]
        @test lift_over_drag(kps) ≈ 4.419990795214735 rtol=2e-3      # initial lift-over-drag
        @test norm(v_wind_kite(kps)) ≈ 9.107670173739065 rtol=1e-2   # initial wind speed at the height of the kite [m/s]
    end

    @testset "test_find_steady_state upwind_dir = $(round(rad2deg(upwind_dir)))°" for upwind_dir in (-π/2, 0.0, π/4, π)
        local kps = KPS3(KCU(load_settings("system.yaml")))
        init_392(kps)
        KiteModels.set_depower_steering!(kps, 0.25, 0.0)
        res1, res2 = @test_logs min_level=Base.CoreLogging.Warn find_steady_state!(kps; upwind_dir)
        @test rad2deg(calc_elevation(kps)) ≈ 70.0 atol=1e-6
        @test winch_force(kps) ≈ 306.73663152733974 rtol = 1e-4   # independent of the wind direction
        # the tether lies in the vertical plane of the wind direction; take the components of the
        # accelerations in this plane (horizontal and vertical) and normal to it
        turnangle = upwind_dir + π/2
        in_plane(x, y) = cos(turnangle) * x - sin(turnangle) * y
        normal(x, y)   = sin(turnangle) * x + cos(turnangle) * y
        segments = kps.set.segments
        j = 3*(segments-1)
        @test abs(normal(res1[j+1], res1[j+2])) < 1e-3          # kite position in the plane
        @test in_plane(res1[j+1], res1[j+2]) > 0                 # downwind of the ground station
        res = zeros(length(res1))
        KiteModels.residual!(res, res2, res1, kps)
        for i in 1:segments
            j = 3*(i-1) + 3*segments
            @test abs(in_plane(res[j+1], res[j+2])) < 1e-5       # horizontal balance in the plane
            i < segments && @test abs(res[j+3]) < 1e-5            # vertical balance of the tether particles
        end
    end

    @testset "test_gravity_correction" begin
        # Steer the kite to the side, release the steering and return how far gravity turns
        # it towards the ground (heading 180°) within 0.5 s.
        function heading_change(steering; c2_cor)
            set = deepcopy(load_settings("system.yaml"))
            set.c2_cor = c2_cor
            kps = KPS3(KCU(set))
            integrator = KiteModels.init!(kps)
            for _ in 1:30
                set_depower_steering(kps.kcu, 0.25, steering)
                next_step!(kps, integrator; set_speed=0, dt=0.05)
            end
            heading_0 = calc_heading(kps)
            set_depower_steering(kps.kcu, 0.25, 0.0)
            next_step!(kps, integrator; set_speed=0, dt=0.05)
            # next_step! uses the heading at the start of the time step for the correction;
            # it is calculated with the accepted state and the wind at its height, so it differs
            # slightly from heading_0
            @test kps.psi ≈ heading_0 atol=1e-3
            for _ in 1:9
                next_step!(kps, integrator; set_speed=0, dt=0.05)
            end
            rad2deg(atan(sin(calc_heading(kps) - heading_0), cos(calc_heading(kps) - heading_0)))
        end
        # if the wind direction changes, the heading is calculated with the new wind
        let set = deepcopy(load_settings("system.yaml"))
            kps = KPS3(KCU(set))
            integrator = something(KiteModels.init!(kps))
            for _ in 1:10
                next_step!(kps, integrator; set_speed=0, dt=0.05, upwind_dir=-pi/2)
            end
            u_0 = copy(integrator.u)
            next_step!(kps, integrator; set_speed=0, dt=0.05, upwind_dir=-pi/2 + deg2rad(30))
            # s.v_wind is the new wind; restore the state at the start of the time step
            KiteModels.update_kite_state!(kps, u_0)
            kps.v_apparent .= kps.v_wind - kps.vel_kite
            @test kps.psi ≈ calc_heading(kps)
        end
        c2_cor = load_settings("system.yaml").c2_cor
        @test c2_cor > 0
        for steering in (0.3, -0.3)
            uncorrected = heading_change(steering; c2_cor=0.0)
            corrected   = heading_change(steering; c2_cor)
            # gravity turns the kite towards the ground, the correction reduces this effect
            @test abs(uncorrected) > 10
            @test sign(corrected) == sign(uncorrected)
            @test abs(corrected) < 0.95 * abs(uncorrected)
        end
    end

end
nothing
