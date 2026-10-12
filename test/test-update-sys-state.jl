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
    ss = SysState(kps4)
    azimuth_0, heading_0, time_0 = ss.azimuth, ss.heading, ss.time
    next_step!(kps4, integrator; set_speed=0, dt=DT)
    # the fresh angles are needed in full precision; the ones stored in `ss` are Float32
    azimuth_1, heading_1 = calc_azimuth(kps4), calc_heading(kps4)
    update_sys_state!(ss, kps4)
    dt = ss.time - time_0
    @test dt ≈ DT
    @test ss.azimuth_rate ≈ wrap(azimuth_1 - azimuth_0) / dt
    @test ss.heading_rate ≈ wrap(heading_1 - heading_0) / dt
    # KPS4: the turn rates are the angular velocity of the kite frame
    @test ss.turn_rates ≈ KiteModels.calc_turn_rates(kps4)
end

@testset "update_sys_state! rates at the wrap boundary" begin
    # A previous sample one full turn away from the current angle is the same angle:
    # the shortest-angle difference must see 0.1 rad, not 0.1 - 2pi.
    ss = SysState(kps4)
    ss.time = kps4.t_0 - DT
    ss.heading = calc_heading(kps4) - 0.1 - 2pi
    ss.azimuth = calc_azimuth(kps4) + 0.2 + 2pi
    update_sys_state!(ss, kps4)
    @test ss.heading_rate ≈ 0.1 / DT
    @test ss.azimuth_rate ≈ -0.2 / DT
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
    # vel_kite is from the last call of residual!, not from the accepted state, so it is not
    # exact; the velocity of a tether particle would differ by tens of percent
    @test kps3.vel_kite ≈ integrator3.u[i_vel+1:i_vel+3] rtol=1e-3
    @test ss.vel_kite ≈ kps3.vel_kite

    azimuth_0, heading_0, time_0 = ss.azimuth, ss.heading, ss.time
    next_step!(kps3, integrator3; set_speed=0, dt=DT)
    azimuth_1, heading_1 = calc_azimuth(kps3), calc_heading(kps3)
    update_sys_state!(ss, kps3)
    dt = ss.time - time_0
    @test ss.azimuth_rate ≈ wrap(azimuth_1 - azimuth_0) / dt
    @test ss.heading_rate ≈ wrap(heading_1 - heading_0) / dt
end

@testset "update_sys_state! KPS3 forces and velocities" begin
    kps3 = KPS3(deepcopy(set))
    integrator3 = init!(kps3; prn=false)
    for _ in 1:20
        next_step!(kps3, integrator3; set_speed=0, dt=DT)
    end
    # evaluate the residual for the accepted state, so that the forces of the residual and of
    # update_sys_state! belong to the same positions and velocities
    KiteModels.residual!(zeros(length(integrator3.u)), copy(integrator3.du), copy(integrator3.u),
                         kps3, integrator3.t)
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
end

nothing
