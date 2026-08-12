# SPDX-FileCopyrightText: 2026 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end
using KiteModels, KiteUtils, LinearAlgebra, StaticArrays, Test
# the following line is needed because VortexStepMethod also exports init!
using KiteModels: init!

set_data_path(joinpath(dirname(dirname(pathof(KiteModels)::String)), "data"))

"Shortest signed difference between two angles, the reference `update_sys_state!` implements."
wrap(delta) = atan(sin(delta), cos(delta))

set = deepcopy(load_settings("system.yaml"))
set.upwind_dir = rad2deg(-pi/2)
kps4::KPS4 = KPS4(deepcopy(set))
integrator = init!(kps4; stiffness_factor=0.5, delta=0.005, prn=false)
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
    @test [ss.CL2, ss.CD2] ≈ collect(cl_cd(kps4))
end

@testset "update_sys_state! vector fields" begin
    ss = SysState(kps4)
    @test length(ss.X) == length(kps4.pos)
    @test ss.X ≈ [pos[1] for pos in kps4.pos]
    @test ss.Y ≈ [pos[2] for pos in kps4.pos]
    @test ss.Z ≈ [pos[3] for pos in kps4.pos]
    @test ss.orient ≈ calc_orient_quat(kps4)
    @test [ss.roll, ss.pitch, ss.yaw] ≈ orient_euler(kps4)
    @test ss.vel_kite ≈ kps4.vel_kite
    @test ss.v_wind_gnd ≈ kps4.v_wind_gnd
    @test ss.v_wind_kite ≈ kps4.v_wind
    @test ss.v_wind_200m ≈ kps4.v_wind_gnd * calc_wind_factor(kps4.am, 200.0)
    # the winch quantities are scalars on a single-tether model, in slot 1 of a 4-tether vector
    @test ss.winch_force ≈ [winch_force(kps4), 0, 0, 0]
    @test ss.l_tether ≈ [kps4.l_tether, 0, 0, 0]
    @test ss.v_reelout ≈ [kps4.v_reel_out, 0, 0, 0]
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
    @test ss.set_speed ≈ [0.5, 0, 0, 0]
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
    @test ss.set_torque ≈ [-20.0, 0, 0, 0]
    @test ss.set_force ≈ [1200.0, 0, 0, 0]
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
    # Erhard and Strauch (2013): the roll contribution is removed from the heading rate
    @test ss.turn_rates ≈ [0, 0, ss.heading_rate - ss.azimuth_rate * sin(ss.elevation)]
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

@testset "update_sys_state! KPS3" begin
    kps3 = KPS3(deepcopy(set))
    integrator3 = init!(kps3; stiffness_factor=0.5, delta=0.0001, prn=false)
    next_step!(kps3, integrator3; set_speed=0, dt=DT)
    ss = SysState(kps3)
    @test length(ss.X) == length(kps3.pos)
    @test ss.elevation ≈ calc_elevation(kps3)
    @test ss.azimuth ≈ calc_azimuth(kps3)
    @test ss.heading ≈ calc_heading(kps3)
    @test ss.l_tether ≈ [kps3.l_tether, 0, 0, 0]
    # set_force, bearing and attractor are KPS4 only and keep their defaults here
    @test ss.bearing == 0
    @test ss.set_force ≈ [0, 0, 0, 0]
    @test ss.attractor ≈ [0, 0]

    azimuth_0, heading_0, time_0 = ss.azimuth, ss.heading, ss.time
    next_step!(kps3, integrator3; set_speed=0, dt=DT)
    azimuth_1, heading_1 = calc_azimuth(kps3), calc_heading(kps3)
    update_sys_state!(ss, kps3)
    dt = ss.time - time_0
    @test ss.azimuth_rate ≈ wrap(azimuth_1 - azimuth_0) / dt
    @test ss.heading_rate ≈ wrap(heading_1 - heading_0) / dt
end

nothing
