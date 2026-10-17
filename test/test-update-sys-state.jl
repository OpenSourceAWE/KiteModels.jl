# SPDX-FileCopyrightText: 2026 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using KiteModels, KiteUtils, LinearAlgebra, Rotations, StaticArrays, Test
# the following line is needed because VortexStepMethod also exports init!
using KiteModels: init!

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

"Shortest signed difference between two angles, the reference `update_sys_state!` implements."
wrap(delta) = atan(sin(delta), cos(delta))

set = deepcopy(load_settings("system.yaml"))
set.upwind_dir = rad2deg(-pi/2)
kps4::KPS4 = KPS4(deepcopy(set))
integrator = init!(kps4; prn=false)
DT = 1/set.sample_freq
# `next_step!` stores the time of the *previous* step in `t_0`, so two steps are needed
# before `s.t_0 > 0` and the rate calculation sees a non-zero time difference
for _ in 1:2
    next_step!(kps4, integrator; set_speed=0, dt=DT)
end

@testset "update_sys_state! scalar fields" begin
    ss = SysState(kps4)
    @test ss.time == kps4.t_0
    @test ss.t_sim == 0.0
    @test ss.elevation ≈ calc_elevation(kps4)
    @test ss.azimuth ≈ calc_azimuth(kps4)
    @test ss.heading ≈ calc_heading(kps4)
    @test ss.course ≈ calc_course(kps4)
    @test ss.v_app ≈ norm(kps4.v_apparent)
    @test ss.depower ≈ kps4.depower
    @test ss.steering ≈ kps4.steering / set.cs_4p
    @test ss.kcu_steering ≈ kps4.kcu_steering / set.cs_4p
    @test ss.set_steering ≈ kps4.kcu.set_steering
    @test ss.AoA ≈ deg2rad(kps4.alpha_2)
    @test ss.alpha3 ≈ deg2rad(kps4.alpha_3)
    @test ss.alpha4 ≈ deg2rad(kps4.alpha_4)
    @test ss.side_slip ≈ kps4.side_slip
    @test [ss.CL2, ss.CD2] ≈ collect(cl_cd(kps4))
end

@testset "update_sys_state! vector fields" begin
    ss = SysState(kps4)
    @test length(ss.X) == length(kps4.pos)
    @test ss.X ≈ [pos[1] for pos in kps4.pos]
    @test ss.Y ≈ [pos[2] for pos in kps4.pos]
    @test ss.Z ≈ [pos[3] for pos in kps4.pos]
    @test ss.VX ≈ [vel[1] for vel in kps4.vel]
    @test ss.VY ≈ [vel[2] for vel in kps4.vel]
    @test ss.VZ ≈ [vel[3] for vel in kps4.vel]
    @test SVector(ss.VX[end-2], ss.VY[end-2], ss.VZ[end-2]) ≈ kps4.vel_kite
    @test ss.orient ≈ fromKS2KA(calc_orient_quat(kps4))
    @test SVector(euler_KS(ss.orient)) ≈ orient_euler(kps4)
    @test ss.vel_kite ≈ kps4.vel_kite
    @test ss.v_wind_gnd ≈ kps4.v_wind_gnd
    @test ss.v_wind_kite ≈ kps4.v_wind
    @test ss.v_wind_200m ≈ kps4.v_wind_gnd * calc_wind_factor(kps4.am, 200.0)
    @test ss.winch_force ≈ [winch_force(kps4)]
    @test ss.l_tether ≈ [kps4.l_tether]
    @test ss.v_reelout ≈ [kps4.v_reel_out]
end

@testset "update_sys_state! spring forces" begin
    ss = SysState(kps4)
    @test length(ss.spring_force) == n_springs(kps4) == set.segments + KiteModels.KITE_SPRINGS
    @test ss.spring_force ≈ spring_forces(kps4; prn=false)
    @test_throws ArgumentError update_sys_state!(SysState(length(kps4.pos); segments=3), kps4)
    logger = Logger(length(kps4.pos), 2; segments=n_springs(kps4))
    log!(logger, ss)
    @test logger.spring_force_vec[1] ≈ ss.spring_force
    # without the spring forces, the SysState fits a Logger without segments
    ss = SysState(kps4; with_springs=false)
    @test isempty(ss.spring_force)
    log!(Logger(length(kps4.pos), 2), ss)
end

@testset "update_sys_state! spring forces include damping" begin
    s = KPS4(deepcopy(set))
    integ = init!(s; prn=false)
    for _ in 1:20
        next_step!(s, integ; set_speed=1.0, dt=DT)
    end
    ss = SysState(s)
    damping = ss.spring_force - spring_forces(s; prn=false, damping=false)
    # the tether segments are in tension, so their damping force is c * elongation rate
    for i in 1:set.segments
        spring = s.springs[i]
        segment = s.pos[spring.p1] - s.pos[spring.p2]
        spring_vel = normalize(segment) ⋅ (s.vel[spring.p1] - s.vel[spring.p2])
        @test norm(segment) > spring.length
        @test damping[i] ≈ spring.axial_damping * spring_vel rtol=1e-4
    end
    @test maximum(abs, damping[1:set.segments]) > 1.0

    kps3 = KPS3(deepcopy(set))
    integ3 = init!(kps3; prn=false)
    for _ in 1:20
        next_step!(kps3, integ3; set_speed=1.0, dt=DT)
    end
    ss3 = SysState(kps3)
    vel = KiteModels.particle_velocities(kps3)
    damping = ss3.spring_force - spring_forces(kps3; damping=false)
    for i in 1:set.segments
        segment = kps3.pos[i+1] - kps3.pos[i]
        spring_vel = normalize(segment) ⋅ (vel[i+1] - vel[i])
        @test norm(segment) > kps3.segment_length
        @test damping[i] ≈ kps3.axial_damping * spring_vel rtol=1e-4
    end
    @test maximum(abs, damping) > 1.0
end

@testset "save_state" begin
    ss = SysState(kps4)
    ss.depower = NaN
    file = joinpath(mktempdir(), "sys_state.yaml")
    save_state(file, ss; meta_data=Dict("script" => "test", "time" => ss.time))
    yaml = KiteUtils.YAML.load_file(file)
    @test yaml["meta_data"] == Dict("script" => "test", "time" => ss.time)
    state = yaml["sys_state"]
    @test !haskey(state, "flap_angle")
    @test length(state) == fieldcount(typeof(ss)) - length(KiteModels.UNUSED_FIELDS)
    @test isnan(state["depower"])
    @test state["time"] == ss.time
    # the shortest representation of Float32 values round-trips
    @test Float32.(state["X"]) == ss.X
    @test Float32.(state["spring_force"]) == ss.spring_force
    @test state["elevation"] isa Float64
    save_state(file, ss; exclude=())
    yaml = KiteUtils.YAML.load_file(file)
    @test isempty(yaml["meta_data"])
    @test length(yaml["sys_state"]) == fieldcount(typeof(ss))
    save_state(file, ss; exclude=KiteModels.UNUSED_FIELDS_KPS3)
    state = KiteUtils.YAML.load_file(file)["sys_state"]
    @test !haskey(state, "side_slip") && !haskey(state, "alpha3") && !haskey(state, "alpha4")
    @test !haskey(state, "aero_moment_KA_x")
    @test length(state) == fieldcount(typeof(ss)) - length(KiteModels.UNUSED_FIELDS_KPS3)
    # keys and values that YAML would misread without quoting
    meta_data = Dict("a: b" => 1, "# comment" => 2, "true" => 3, "- item" => 4, :symbol => 5,
                     "text" => "cost \$5, \"quoted\", back\\slash\nnew line")
    save_state(file, ss; meta_data)
    @test KiteUtils.YAML.load_file(file)["meta_data"] ==
          Dict(string(key) => value for (key, value) in meta_data)
end

@testset "load_state" begin
    file = joinpath(mktempdir(), "sys_state.yaml")
    meta_data = Dict("script" => "test", "time" => 1.5)
    # every field round-trips, including NaN, the integers and the Float32 vectors
    ss = SysState(kps4)
    save_state(file, ss; meta_data, exclude=())
    ss2, meta_data2 = load_state(file)
    @test typeof(ss2) == typeof(ss)
    @test all(isequal(getfield(ss2, name), getfield(ss, name)) for name in fieldnames(typeof(ss)))
    @test meta_data2 == meta_data
    # excluded fields keep their default values; KPS3 has fewer points and springs
    kps3 = KPS3(deepcopy(set))
    init!(kps3; prn=false)
    ss = SysState(kps3)
    save_state(file, ss; exclude=KiteModels.UNUSED_FIELDS_KPS3)
    ss2, meta_data2 = load_state(file)
    @test typeof(ss2) == typeof(ss)
    @test ss2.X == ss.X && ss2.spring_force == ss.spring_force
    @test isnan(ss2.bearing)
    @test ss2.alpha3 == 0
    @test isempty(meta_data2)
    # a vector of the wrong length is rejected; the number of points is taken from X
    write(file, replace(read(file, String), r"  Y: \[[^\]]*\]" => "  Y: [1.0]"))
    @test_throws ArgumentError load_state(file)
end

@testset "init! from a loaded state, $Model" for (Model, exclude) in
        ((KPS4, KiteModels.UNUSED_FIELDS), (KPS3, KiteModels.UNUSED_FIELDS_KPS3))
    run!(s, integ, steps) = for _ in 1:steps
        set_depower_steering(s.kcu, 0.25, 0.2)
        next_step!(s, integ; set_speed=0, dt=DT)
    end
    set_ = deepcopy(set)
    if Model == KPS3
        # with the default tolerance of 1e-3, a turning KPS3 is only accurate to about half a
        # metre after one second, and the restarted solver takes different steps than the
        # uninterrupted one; with 1e-6 both follow the exact solution within a millimetre
        set_.rel_tol = set_.abs_tol = 1e-6
    end
    a = Model(deepcopy(set_))
    integ_a = init!(a; prn=false)
    run!(a, integ_a, 40)
    file = joinpath(mktempdir(), "sys_state.yaml")
    ss = SysState(a)
    ss.time = integ_a.t
    save_state(file, ss; exclude)
    b = Model(deepcopy(set_))
    integ_b = init!(b; initial_state=first(load_state(file)))
    if Model == KPS4
        # the pitch rate from the velocities is close to the change of the pitch angle during
        # the last time step, which the uninterrupted simulation uses
        @test b.pitch_rate ≈ KiteModels.calc_pitch_rate(a) rtol=1e-4
        @test b.pitch_rate ≈ a.pitch_rate rtol=0.1
    end
    # the simulation continues at the same time and stays within a millimetre of the original
    run!(a, integ_a, 20)
    run!(b, integ_b, 20)
    @test integ_b.t ≈ integ_a.t
    @test maximum(norm.(a.pos .- b.pos)) < 1e-3
    @test winch_force(b) ≈ winch_force(a) rtol=1e-4
    # a state with a different number of points is rejected
    other = Model == KPS4 ? KPS3(deepcopy(set)) : KPS4(deepcopy(set))
    @test_throws ArgumentError init!(other; initial_state=SysState(a))
end

@testset "init! from a loaded state during a winch ramp" begin
    step!(s, integ, v_set) = next_step!(s, integ; set_speed=v_set, dt=DT)
    a = KPS4(deepcopy(set))
    integ_a = init!(a; prn=false)
    for _ in 1:40
        step!(a, integ_a, 0)
    end
    # save while the rate limit of the winch is still ramping up the set speed
    for _ in 1:3
        step!(a, integ_a, 4)
    end
    @test 0 < a.wm.last_set_speed < 4
    file = joinpath(mktempdir(), "sys_state.yaml")
    ss = SysState(a)
    ss.time = integ_a.t
    save_state(file, ss)
    b = KPS4(deepcopy(set))
    integ_b = init!(b; initial_state=first(load_state(file)))
    # var_13 and set_speed are Float32
    @test b.wm.last_set_speed ≈ a.wm.last_set_speed rtol=1e-6
    @test b.wm.brake == a.wm.brake
    @test b.sync_speed ≈ a.sync_speed rtol=1e-6
    # without the state of the winch, the tether length would differ by half a metre
    for _ in 1:20
        step!(a, integ_a, 4)
        step!(b, integ_b, 4)
    end
    @test b.l_tether ≈ a.l_tether atol=1e-3
    @test b.v_reel_out ≈ a.v_reel_out atol=1e-3
end

@testset "update_sys_state! zoom" begin
    ss = SysState(kps4)
    zoomed = SysState(kps4, 2.0)
    @test zoomed.X ≈ 2 .* ss.X
    @test zoomed.Y ≈ 2 .* ss.Y
    @test zoomed.Z ≈ 2 .* ss.Z
    @test zoomed.elevation ≈ ss.elevation
end

@testset "update_sys_state! set values" begin
    ss = SysState(kps4)
    next_step!(kps4, integrator; set_speed=0.5, dt=DT)
    update_sys_state!(ss, kps4)
    @test ss.set_speed ≈ [0.5]
    # a set value of `nothing` is logged as NaN, not as zero
    @test isnan(ss.set_torque[1])
    @test isnan(ss.set_force[1])
    @test isnan(ss.bearing)
    @test all(isnan, ss.attractor)

    kps4.sync_speed = nothing
    kps4.set_torque = -20.0
    kps4.set_force  = 1200.0
    kps4.bearing    = 0.3
    kps4.attractor  = SVector(0.1, 0.7)
    update_sys_state!(ss, kps4)
    @test isnan(ss.set_speed[1])
    @test ss.set_torque ≈ [-20.0]
    @test ss.set_force ≈ [1200.0]
    @test ss.bearing ≈ 0.3
    @test ss.attractor ≈ [0.1, 0.7]
end

@testset "update_sys_state! rates" begin
    SysState(kps4)
    # the previous angles and their time are kept in the model, in full precision
    azimuth_0, heading_0, time_0 = kps4.azimuth_last, kps4.heading_last, kps4.t_last
    @test azimuth_0 ≈ calc_azimuth(kps4)
    @test heading_0 ≈ calc_heading(kps4)
    @test time_0 == integrator.t
    next_step!(kps4, integrator; set_speed=0, dt=DT)
    ss = SysState(kps4)
    # read the fresh angles after update_sys_state!, which evaluates the model for the
    # accepted state
    azimuth_1, heading_1 = calc_azimuth(kps4), calc_heading(kps4)
    dt = kps4.t_last - time_0
    @test dt ≈ DT
    @test ss.azimuth_rate ≈ wrap(azimuth_1 - azimuth_0) / dt
    @test ss.heading_rate ≈ wrap(heading_1 - heading_0) / dt
    @test ss.azimuth_rate != 0
    @test ss.heading_rate != 0
    # KPS4: the turn rates are the angular velocity of the kite frame
    @test ss.turn_rates ≈ KiteModels.calc_turn_rates(kps4)
end

@testset "update_sys_state! rates at the wrap boundary" begin
    # A previous sample one full turn away from the current angle is the same angle:
    # the shortest-angle difference must see 0.1 rad, not 0.1 - 2pi.
    SysState(kps4)
    kps4.t_last = integrator.t - DT
    kps4.heading_last = calc_heading(kps4) - 0.1 - 2pi
    kps4.azimuth_last = calc_azimuth(kps4) + 0.2 + 2pi
    ss = SysState(kps4)
    @test ss.heading_rate ≈ 0.1 / DT
    @test ss.azimuth_rate ≈ -0.2 / DT
end

@testset "update_sys_state! rates without time step" begin
    # a further update at the same time keeps the rates, also with a new SysState, instead of
    # dividing by a zero time difference
    ss_0 = SysState(kps4)
    ss = SysState(kps4)
    @test ss.heading_rate == ss_0.heading_rate
    @test ss.azimuth_rate == ss_0.azimuth_rate
    @test isfinite(ss.heading_rate)
    @test isfinite(ss.azimuth_rate)
    # KPS4: the turn rates are the angular velocity of the kite frame, which needs no time step
    @test ss.turn_rates ≈ KiteModels.calc_turn_rates(kps4)
    @test all(isfinite, ss.turn_rates)
    # the first update after init! has no previous angles, so the rates are zero
    s = KPS4(deepcopy(set))
    init!(s; prn=false)
    ss = SysState(s)
    @test ss.heading_rate == 0
    @test ss.azimuth_rate == 0
end

@testset "update_sys_state! rates with a new SysState in each step" begin
    # a fresh SysState starts with the angles and the time zero; reading the previous angles
    # from it gave the angles divided by t_0
    s = KPS4(deepcopy(set))
    integ = init!(s; prn=false)
    ss_reused = SysState(s)
    set_depower_steering(s.kcu, 0.25, 0.3)
    for _ in 1:20
        next_step!(s, integ; set_speed=0, dt=DT)
        update_sys_state!(ss_reused, s)
        ss_new = SysState(s)
        @test ss_new.heading_rate == ss_reused.heading_rate
        @test ss_new.azimuth_rate == ss_reused.azimuth_rate
    end
    @test abs(ss_reused.heading_rate) > 0.1
    @test abs(ss_reused.heading_rate) < 10
end

@testset "update_sys_state! aerodynamic force and moment" begin
    kps4.steering = 0.0
    next_step!(kps4, integrator; set_speed=0, dt=DT)
    ss = SysState(kps4)
    force  = SVector(ss.aero_force_KA_x[1],  ss.aero_force_KA_y[1],  ss.aero_force_KA_z[1])
    moment = SVector(ss.aero_moment_KA_x[1], ss.aero_moment_KA_y[1], ss.aero_moment_KA_z[1])
    # the KA axes in ENU are aft, right and up; the forces of KPS4 have the opposite sign
    # of the physical forces
    R_KA = hcat(-kps4.x, kps4.y, -kps4.z)
    @test R_KA * force ≈ -(kps4.lift_force + kps4.drag_force + kps4.side_force) rtol=1e-5
    # lift is up, drag is aft
    @test force[3] > 0
    @test force[1] > 0
    # without steering the kite is symmetric: no roll and no yaw moment
    @test abs(moment[1]) < 1e-3 * norm(moment)
    @test abs(moment[3]) < 1e-3 * norm(moment)

    # the aerodynamic forces act on the kite particles A, B, C and D only
    aero_force = [SVector(ss.aero_force_x[i], ss.aero_force_y[i], ss.aero_force_z[i])
                  for i in eachindex(ss.X)]
    i_A = set.segments + 2
    @test all(iszero, aero_force[1:i_A-1])
    @test sum(aero_force[i_A:i_A+3]) ≈ R_KA * force rtol=1e-5
    # the moment is taken about the origin of the KA frame, the centre between C and D
    pos = [SVector{3}(p) for p in kps4.pos]
    centre = 0.5 * (pos[i_A+2] + pos[i_A+3])
    moment_ENU = sum((pos[i] - centre) × aero_force[i] for i in i_A:i_A+3)
    @test R_KA * moment ≈ moment_ENU rtol=1e-4
end

@testset "update_sys_state! drag forces" begin
    ss = SysState(kps4)
    drag_force = [SVector(ss.drag_force_x[i], ss.drag_force_y[i], ss.drag_force_z[i])
                  for i in eachindex(ss.X)]
    # the spring forces of inner_loop! cancel in pairs, so the sum of its forces is the
    # sum of the drag forces, with the opposite sign
    P = length(kps4.pos)
    pos = SVector{P}(SVector{3}(p) for p in kps4.pos)
    vel = SVector{P}(SVector{3}(v) for v in kps4.vel)
    for force in kps4.forces
        force .= 0
    end
    KiteModels.inner_loop!(kps4, pos, vel, kps4.v_wind_gnd, set.segments, set.d_tether/1000)
    @test sum(drag_force) ≈ -sum(SVector{3}.(kps4.forces)) rtol=1e-5
    @test all(!iszero, drag_force)
end

@testset "update_sys_state! acceleration" begin
    s = KPS4(deepcopy(set))
    integ = init!(s; prn=false)
    @test SysState(s).acc == 0
    for _ in 1:20
        next_step!(s, integ; set_speed=0, dt=DT)
    end
    # the kite velocity is part of the state vector, after the positions of the particles 2:P
    P = length(s.pos)
    i_vel = 3 * (P - 1) + 3 * (P - 4)
    vel_kite(integ) = integ.u[i_vel+1:i_vel+3]
    vel_1 = vel_kite(integ)
    next_step!(s, integ; set_speed=0, dt=DT)
    acc = copy(s.acc_kite)
    next_step!(s, integ; set_speed=0, dt=DT)
    vel_3 = vel_kite(integ)
    # central difference
    @test acc ≈ (vel_3 - vel_1) / (2DT) rtol=0.05
    @test norm(acc) > 0.1
    @test SysState(s).acc ≈ norm(s.acc_kite)
end

@testset "update_sys_state! turn rates" begin
    s = KPS4(deepcopy(set))
    integ = init!(s; prn=false)
    step!() = next_step!(s, integ; set_speed=0, dt=DT)
    # rotation matrix of the kite in the KS convention, with the axes in ENU
    rot() = hcat(SVector{3}(s.x), SVector{3}(s.y), SVector{3}(s.z))
    set_depower_steering(s.kcu, 0.25, 0.3)
    for _ in 1:60
        step!()
    end
    rot_0 = rot()
    step!()
    rot_1 = rot()
    ss = SysState(s)
    side_slip = s.side_slip
    step!()
    rot_2 = rot()
    # central difference of the rotation matrix; Ω is the skew matrix of the body rates
    Ω = rot_1' * (rot_2 - rot_0) / (2DT)
    turn_rates = fromKS2KA_body(SVector(Ω[3, 2], Ω[1, 3], Ω[2, 1]))
    @test SVector(ss.turn_rate_x[1], ss.turn_rate_y[1], ss.turn_rate_z[1]) ≈ turn_rates rtol=0.01
    @test ss.turn_rates ≈ turn_rates rtol=0.01
    @test norm(turn_rates) > 0.1
    # in a turn the apparent wind comes from the side
    @test ss.side_slip ≈ side_slip
    @test abs(ss.side_slip) > 0.01
end

@testset "update_sys_state! KPS3" begin
    kps3 = KPS3(deepcopy(set))
    integrator3 = init!(kps3; prn=false)
    next_step!(kps3, integrator3; set_speed=0, dt=DT)
    ss = SysState(kps3)
    @test length(ss.X) == length(kps3.pos)
    @test ss.elevation ≈ calc_elevation(kps3)
    @test ss.azimuth ≈ calc_azimuth(kps3)
    @test ss.heading ≈ calc_heading(kps3)
    @test ss.l_tether ≈ [kps3.l_tether]
    # side_slip, alpha3, alpha4, set_force, bearing and attractor are KPS4 only and marked as
    # not available
    @test isnan(ss.side_slip)
    @test isnan(ss.alpha3)
    @test isnan(ss.alpha4)
    @test isnan(ss.bearing)
    @test isnan(ss.set_force[1])
    @test all(isnan, ss.attractor)
    @test SysState(kps3).spring_force ≈ spring_forces(kps3)
    # a point mass has no aerodynamic moment
    @test ss.aero_moment_KA_y ≈ [0]
    # the first update has no previous orientation for the turn rates
    @test all(iszero, ss.turn_rates)
    # KPS3 stores the acceleration of the kite like KPS4
    @test ss.acc ≈ norm(kps3.acc_kite)
    # the kite is the last particle; its velocity follows the positions of the particles 2:P
    P = length(kps3.pos)
    i_vel = 3 * (P - 1) + 3 * (P - 2)
    # SysState evaluates the residual for the accepted state, so vel_kite belongs to it
    @test kps3.vel_kite ≈ integrator3.u[i_vel+1:i_vel+3]
    @test ss.vel_kite ≈ kps3.vel_kite

    # the previous angles are kept in the model in full precision
    azimuth_0, heading_0, time_0 = kps3.azimuth_last, kps3.heading_last, kps3.t_last
    next_step!(kps3, integrator3; set_speed=0, dt=DT)
    update_sys_state!(ss, kps3)
    azimuth_1, heading_1 = calc_azimuth(kps3), calc_heading(kps3)
    dt = kps3.t_last - time_0
    @test dt ≈ DT
    @test ss.azimuth_rate ≈ wrap(azimuth_1 - azimuth_0) / dt atol=1e-6
    @test ss.heading_rate ≈ wrap(heading_1 - heading_0) / dt atol=1e-6
end

@testset "update_sys_state! KPS3 forces and velocities" begin
    kps3 = KPS3(deepcopy(set))
    integrator3 = init!(kps3; prn=false)
    for _ in 1:20
        next_step!(kps3, integrator3; set_speed=0, dt=DT)
    end
    # update_sys_state! evaluates the residual for the accepted state, so afterwards the forces
    # of the residual and of the SysState belong to the same positions and velocities
    ss = SysState(kps3)
    P = length(kps3.pos)
    vel = [SVector(ss.VX[i], ss.VY[i], ss.VZ[i]) for i in 1:P]
    @test vel[1] == zeros(3)
    @test vel[P] ≈ kps3.vel_kite
    i_vel = 3 * (P - 1)
    @test vel[2] ≈ integrator3.u[i_vel+1:i_vel+3]

    drag_force = [SVector(ss.drag_force_x[i], ss.drag_force_y[i], ss.drag_force_z[i]) for i in 1:P]
    aero_force = [SVector(ss.aero_force_x[i], ss.aero_force_y[i], ss.aero_force_z[i]) for i in 1:P]
    @test all(iszero, aero_force[1:P-1])
    # lift is up, drag is downwind
    @test aero_force[P][3] > 0
    @test aero_force[P] ⋅ kps3.v_wind > 0
    # the spring forces cancel in pairs, so the forces of the residual on all particles,
    # including the winch force on the first one, sum up to the drag and the aerodynamic force;
    # the forces of KPS3 have the opposite sign of the physical forces
    total = sum(SVector{3}.(kps3.forces[2:P])) + kps3.last_force
    @test sum(drag_force) + aero_force[P] ≈ -total rtol=1e-5
    @test all(!iszero, drag_force)

    # the KA axes in ENU are aft, right and up
    x, y, z = KiteModels.kite_ref_frame(kps3)
    force_KA = SVector(ss.aero_force_KA_x[1], ss.aero_force_KA_y[1], ss.aero_force_KA_z[1])
    @test hcat(-x, y, -z) * force_KA ≈ aero_force[P] rtol=1e-5
end

@testset "update_sys_state! KPS3 turn rates" begin
    kps3 = KPS3(deepcopy(set))
    integrator3 = init!(kps3; prn=false)
    ss = SysState(kps3)
    set_depower_steering(kps3.kcu, 0.25, 0.3)
    for _ in 1:40
        next_step!(kps3, integrator3; set_speed=0, dt=DT)
        update_sys_state!(ss, kps3)
    end
    orient_0 = QuatRotation(fromKS2KA(calc_orient_quat(kps3))...)
    next_step!(kps3, integrator3; set_speed=0, dt=DT)
    update_sys_state!(ss, kps3)
    orient_1 = QuatRotation(fromKS2KA(calc_orient_quat(kps3))...)
    rotation = RotationVec(orient_0' * orient_1)
    turn_rates = SVector(rotation.sx, rotation.sy, rotation.sz) / DT
    @test ss.turn_rates ≈ turn_rates rtol=1e-3
    @test SVector(ss.turn_rate_x[1], ss.turn_rate_y[1], ss.turn_rate_z[1]) ≈ ss.turn_rates
    @test norm(turn_rates) > 0.1
    # a second update in the same time step keeps the turn rates
    update_sys_state!(ss, kps3)
    @test ss.turn_rates ≈ turn_rates rtol=1e-3
end

@testset "update_sys_state! KPS3 turn rates with a new SysState in each step" begin
    # the previous orientation is kept in the model, not in the SysState, so the result does
    # not depend on whether the SysState is reused
    kps3 = KPS3(deepcopy(set))
    integrator3 = init!(kps3; prn=false)
    ss_reused = SysState(kps3)
    set_depower_steering(kps3.kcu, 0.25, 0.3)
    orient_prev = QuatRotation(fromKS2KA(calc_orient_quat(kps3))...)
    for i in 1:40
        next_step!(kps3, integrator3; set_speed=0, dt=DT)
        update_sys_state!(ss_reused, kps3)
        ss_new = SysState(kps3)
        @test ss_new.turn_rates ≈ ss_reused.turn_rates
        orient = QuatRotation(fromKS2KA(calc_orient_quat(kps3))...)
        # a fresh SysState starts at the identity orientation and time zero; reading the
        # previous orientation from it gave the absolute attitude divided by t_0
        rotation = RotationVec(orient_prev' * orient)
        turn_rates = SVector(rotation.sx, rotation.sy, rotation.sz) / DT
        @test ss_new.turn_rates ≈ turn_rates rtol=1e-3 atol=1e-4
        orient_prev = orient
    end
    @test norm(ss_reused.turn_rates) > 0.1
end

nothing
