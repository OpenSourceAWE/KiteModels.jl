# Copyright (c) 2020, 2021, 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

#= Model of a kite-power system in implicit form: residual = f(y, yd)

This model implements a 3D mass-spring system with reel-out. It uses six tether segments (the number can be
configured in the file data/settings.yaml). The kite is modelled using 4 point masses and 3 aerodynamic
surfaces. The spring constant and the axial_damping decrease with the segment length. The aerodynamic kite forces
are acting on three of the four kite point masses.

Four point kite model, included from KiteModels.jl.

Scientific background: http://arxiv.org/abs/1406.6218 =#

# Array of connections of bridlepoints.
# First point, second point, unstressed length.
const SPRINGS_INPUT = [0.    1.  150.
                       1.    2.   -1. # s1, p7, p8
                       4.    2.   -1. # s2, p10, p8
                       4.    5.   -1. # s3, p10, p11
                       3.    4.   -1. # s4, p9, p10
                       5.    1.   -1. # s5, p11, p7
                       4.    1.   -1. # s6, p10, p7
                       3.    5.   -1. # s7, p9, p11
                       5.    2.   -1. # s8, p11, p8
                       2.    3.   -1.] # s9, p8, p9

# KCU = p7, A = p8, B = p9, C = p10, D = p11

# struct, defining the physical parameters of one spring
@with_kw struct Spring{I, S}
    p1::I = 1         # number of the first point
    p2::I = 2         # number of the second point
    length::S = 1.0   # current unstressed spring length
    axial_stiffness::S = 1.0 # spring constant [N/m]
    axial_damping::S  = 0.1 # axial_damping coefficient [Ns/m]
end

const SP = Spring{Int16, SimFloat}
const KITE_PARTICLES = 4
const KITE_SPRINGS = 9
const DRAG_CORR = 0.93       # correction of the drag for the 4-point model
function zero(::Type{SP})
    SP(0,0,0,0,0)
end

"""
    mutable struct KPS4{S, T, P, Q, SP} <: AbstractKiteModel

State of the kite power system, using a 4 point kite model. Parameters:
- S: Scalar type, e.g. SimFloat
  In the documentation mentioned as Any, but when used in this module it is always SimFloat and not Any.
- T: Vector type, e.g. MVector{3, SimFloat}
- P: number of points of the system, segments+1
- Q: number of springs in the system, P-1
- SP: struct type, describing a spring
Normally a user of this package will not have to access any of the members of this type directly,
use the input and output functions instead.

$(TYPEDFIELDS)
"""
@with_kw mutable struct KPS4{S, T, P, Q, SP} <: AbstractKiteModel
    "Reference to the settings struct"
    set::Settings
    "Reference to the KCU model (Kite Control Unit as implemented in the package KitePodModels"
    kcu::KCU
    "Reference to the atmospheric model as implemented in the package AtmosphericModels"
    am::AtmosphericModel = AtmosphericModel(set)
    "Reference to winch model as implemented in the package WinchModels"
    wm::WinchModels.AbstractWinchModel
    "Integrator, storing the current state"
    integrator::Union{OrdinaryDiffEqCore.ODEIntegrator, Sundials.IDAIntegrator, Nothing} = nothing
    "Iterations, number of calls to the function residual!"
    iter:: Int64 = 0
    "Function for calculation the lift coefficient, using a spline based on the provided value pairs."
    calc_cl::Spline1D
    "Function for calculation the drag coefficient, using a spline based on the provided value pairs."
    calc_cd::Spline1D
    "wind vector at the height of the kite"
    v_wind::T =           zeros(S, 3)
    "wind vector at reference height"
    v_wind_gnd::T =       zeros(S, 3)
    "wind vector used for the calculation of the tether drag"
    v_wind_tether::T =    zeros(S, 3)
    "vertical wind velocity at the kite, positive up [m/s]"
    v_wind_vert::S =      0.0
    "apparent wind vector at the kite"
    v_apparent::T =       zeros(S, 3)
    "bridle_factor = set.l_bridle/bridle_length(set)"
    bridle_factor::S = 1.0
    "side lift coefficient, the difference of the left and right lift coefficients"
    side_cl::S = 0.0
    "drag force of kite and bridle; output of calc_aero_forces!"
    drag_force::T =       zeros(S, 3)
    "side_force acting on the kite"
    side_force::T =       zeros(S, 3)
    f_d::T = zeros(S, 3)
    f_s::T = zeros(S, 3)
    "max_steering angle in radian"
    ks::S =               0.0
    "lift force of the kite; output of calc_aero_forces!"
    lift_force::T =       zeros(S, 3)
    "spring force of the current tether segment, output of calc_particle_forces!"
    spring_force::T =     zeros(S, 3)
    "last winch force"
    last_force::T =       zeros(S, 3)
    "a copy of the residual one (pos,vel) for debugging and unit tests"
    res1::SVector{P, KVec3} = zeros(SVector{P, KVec3})
    "a copy of the residual two (vel,acc) for debugging and unit tests"
    res2::SVector{P, KVec3} = zeros(SVector{P, KVec3})
    "a copy of the actual positions as output for the user"
    pos::SVector{P, KVec3} = zeros(SVector{P, KVec3})
    "a copy of the actual velocities as output for the user"
    vel::SVector{P, KVec3} = zeros(SVector{P, KVec3})
    "velocity vector of the kite"
    vel_kite::T =          zeros(S, 3)
    "acceleration vector of the kite, updated once per time step by next_step!"
    acc_kite::T =          zeros(S, 3)
    "time of the last update of the rates by update_sys_state! [s]; -Inf before the first one"
    t_last::S =            -Inf
    "azimuth [rad] at the time `t_last`"
    azimuth_last::S =      zero(S)
    "heading [rad] at the time `t_last`"
    heading_last::S =      zero(S)
    "azimuth rate [rad/s], updated by update_sys_state!"
    azimuth_rate::S =      zero(S)
    "heading rate [rad/s], updated by update_sys_state!"
    heading_rate::S =      zero(S)
    "unstressed segment length [m]"
    segment_length::S =           0.0
    "lift coefficient of the kite, depending on the angle of attack"
    param_cl::S =         0.2
    "drag coefficient of the kite, depending on the angle of attack"
    param_cd::S =         1.0
    "azimuth angle in radian; initial value is zero"
    psi::S =              zero(S)
    "depower angle [deg]"
    alpha_depower::S =     0.0
    "pitch angle [rad]"
    pitch::S =            0.0
    "pitch rate [rad/s]"
    pitch_rate::S =       0.0
    "aoa at particle B"
    alpha_2::S =           0.0
    "aoa at particle C"
    alpha_3::S =           0.0
    "aoa at particle D"
    alpha_4::S =           0.0
    "side_slip angle [rad]"
    side_slip::S = 0.0
    "relative start time of the current time interval"
    t_0::S =               0.0
    "reel out speed of the winch"
    v_reel_out::S =        0.0
    "reel out speed at the last time step"
    last_v_reel_out::S =   0.0
    "unstretched tether length"
    l_tether::S =          0.0
    "air density at the height of the kite"
    rho::S =               0.0
    "actual relative depower setting,  must be between    0 .. 1.0"
    depower::S =           0.0
    "actual relative steering setting, must be between -1.0 .. 1.0"
    steering::S =          0.0
    "steering after the kcu, before applying offset and depower sensitivity, -1.0 .. 1.0"
    kcu_steering::S =      0.0
    "multiplier for the stiffness of tether and bridle"
    stiffness_factor::S =  1.0
    "initial masses of the point masses"
    initial_masses::MVector{P, S} = ones(P)
    "current masses, depending on the total tether length"
    masses::MVector{P, S}         = zeros(P)
    "vector of the springs, defined as struct"
    springs::MVector{Q, SP}       = zeros(SP, Q)
    "vector of the forces, acting on the particles"
    forces::SVector{P, KVec3} = zeros(SVector{P, KVec3})
    "synchronous speed of the motor/ generator"
    sync_speed::Union{S, Nothing} =        0.0
    "set_torque of the motor/generator"
    set_torque::Union{S, Nothing} = nothing
    "set value of the force at the winch, for logging only"
    set_force::Union{S, Nothing} = nothing
    "set value of the bearing angle in radians, for logging only"
    bearing::Union{S, Nothing} = nothing
    "coordinates of the attractor point [azimuth, elevation] in radian, for logging only"
    attractor::Union{SVector{2, S}, Nothing} = nothing
    "x vector of kite reference frame"
    x::T =                 zeros(S, 3)
    "y vector of kite reference frame"
    y::T =                 zeros(S, 3)
    "z vector of kite reference frame"
    z::T =                 zeros(S, 3)
end

@inline @inbounds function norm(vec::SVector{3, SimFloat})
    sqrt(vec[1]*vec[1]+vec[2]*vec[2]+vec[3]*vec[3])
end
@inline @inbounds function norm(vec::MVector{3, SimFloat})
    sqrt(vec[1]*vec[1]+vec[2]*vec[2]+vec[3]*vec[3])
end

"""
    clear!(s::KPS4)

Initialize the kite power model.
"""
function clear!(s::KPS4)
    s.t_0 = 0.0                              # relative start time of the current time interval
    s.iter = 0
    s.v_reel_out = s.set.v_reel_out
    s.last_v_reel_out = s.set.v_reel_out
    s.sync_speed = s.set.v_reel_out
    s.v_wind_gnd    .= [Float64(s.set.v_wind), 0, 0]    # wind vector at reference height
    s.v_wind_tether .= [Float64(s.set.v_wind), 0, 0]
    s.v_wind_vert   = 0.0
    s.v_apparent    .= [Float64(s.set.v_wind), 0, 0]
    height = sin(deg2rad(s.set.elevation::Float64)) * (s.set.l_tethers[1])
    s.v_wind .= s.v_wind_gnd * calc_wind_factor(s.am, height)

    s.l_tether = s.set.l_tethers[1]
    s.segment_length = s.l_tether / s.set.segments
    init_masses!(s)
    init_springs!(s)
    for i in 1:s.set.segments + KiteModels.KITE_PARTICLES + 1
        s.forces[i] .= zeros(3)
    end
    s.drag_force .= [0.0, 0, 0]
    s.lift_force .= [0.0, 0, 0]
    s.side_force .= [0.0, 0, 0]
    s.f_d .= [0.0, 0, 0]
    s.f_s .= [0.0, 0, 0]
    s.spring_force .= [0.0, 0, 0]
    s.last_force .= [0.0, 0, 0]
    s.vel_kite .= [0.0, 0, 0]
    s.acc_kite .= [0.0, 0, 0]
    s.t_last = -Inf
    s.azimuth_rate = 0.0
    s.heading_rate = 0.0
    s.x .= [0.0, 0, 0]
    s.y .= [0.0, 0, 0]
    s.z .= [0.0, 0, 0]
    for i in eachindex(s.pos)
        s.pos[i] .= zeros(3)
        s.vel[i] .= zeros(3)
        s.res1[i] .= zeros(3)
        s.res2[i] .= zeros(3)
    end
    s.side_cl = 0.0
    s.side_slip = 0.0
    s.alpha_2 = 0.0
    s.alpha_3 = 0.0
    s.alpha_4 = 0.0
    s.param_cl = 0.2
    s.param_cd = 1.0
    s.psi = 0.0
    s.rho = s.set.rho_0
    s.bridle_factor = s.set.l_bridle / bridle_length(s.set)
    s.ks = deg2rad(Float64(s.set.max_steering))
    s.kcu.depower = s.set.depower::Float64/100.0
    s.kcu.set_depower = s.kcu.depower
    s.kcu.steering = 0.0
    s.kcu.set_steering = 0.0
    s.pitch = 0.0
    s.pitch_rate = 0.0
    s.alpha_depower = 0.0
    s.depower = 0.0
    s.steering = 0.0
    s.kcu_steering = 0.0
    s.set_torque = nothing
    s.set_force = nothing
    s.bearing = nothing
    s.attractor = nothing
    KiteModels.set_depower_steering!(s, get_depower(s.kcu), get_steering(s.kcu))
end

"""
    KPS4(kcu::KCU)

Create and initialize a four-point kite power system model from an existing
KCU (kite control unit) instance.

This constructor selects the winch model based on `kcu.set.winch_model`,
creates interpolation splines for lift/drag coefficients, and calls `clear!`
to initialize all state variables.

Returns a `KPS4` instance ready for `init!`/`find_steady_state!` and simulation.
"""
function KPS4(kcu::KCU)
    wm = if kcu.set.winch_model == "AsyncMachine"
        AsyncMachine(kcu.set)
    elseif kcu.set.winch_model == "TorqueControlledMachine"
        TorqueControlledMachine(kcu.set)
    end
    # wm.last_set_speed = kcu.set.v_reel_out
    s = KPS4{SimFloat, KVec3, kcu.set.segments+KITE_PARTICLES+1, kcu.set.segments+KITE_SPRINGS, SP}(set=kcu.set,
             kcu=kcu, wm=wm, calc_cl = Spline1D(kcu.set.alpha_cl, kcu.set.cl_list),
             calc_cd=Spline1D(kcu.set.alpha_cd, kcu.set.cd_list) )
    clear!(s)
    return s
end
function KPS4(set::Settings)
    kcu = KCU(set)
    KPS4(kcu)
end

"""
    calc_drag_forces(s::KPS4, unit_vector, norm1, vel1, vel2, v_wind_tether, segments, d_tether,
                     rho, i)

Drag force of the tether segment or bridle line `i` with the direction `unit_vector` and the
length `norm1`, half of which acts on each of its two particles, and the drag force of the KCU,
which is zero except for the last tether segment (`i == segments`). Like `s.forces`, both have
the opposite sign of the physical forces.

Returns a tuple `(half_drag_force, kcu_drag_force)`.
"""
@inline function calc_drag_forces(s::KPS4, unit_vector, norm1, vel1, vel2, v_wind_tether, segments,
                                  d_tether, rho, i)
    v_apparent = v_wind_tether - 0.5 * (vel1 + vel2)
    if s.set.version == 1
        area = norm1 * d_tether
    else
        if i > segments
            area = norm1 * s.set.d_line * 0.001 * s.bridle_factor # 6.0 = A_real/A_simulated
        else
            area = norm1 * d_tether
        end
    end

    v_app_perp = v_apparent - v_apparent ⋅ unit_vector * unit_vector
    half_drag_force = (-0.25 * rho * s.set.cd_tether * norm(v_app_perp) * area) * v_app_perp
    if i == segments
        v_app_kcu = v_wind_tether - vel2
        v_app_perp_kcu = v_app_kcu - v_app_kcu ⋅ unit_vector * unit_vector
        kcu_area = π * (s.set.kcu_diameter/2)^2
        kcu_drag_force = (-0.25 * rho * s.set.cd_kcu * norm(v_app_perp_kcu) * kcu_area) * v_app_perp_kcu
    else
        kcu_drag_force = zero(SVec3)
    end
    half_drag_force, kcu_drag_force
end

"""
    calc_spring_force(s::KPS4, spring, norm1, spring_vel, segments, i; damping=true)

Scalar force of the spring `i` with the length `norm1` [m] and the elongation rate
`spring_vel` [m/s], positive for tension: the elastic force, with a reduced stiffness under
compression, plus the damping force. The residual and [`spring_forces`](@ref) use it.
"""
@inline function calc_spring_force(s::KPS4, spring, norm1, spring_vel, segments, i; damping=true)
    l_0 = spring.length                             # Unstressed length
    k = spring.axial_stiffness * s.stiffness_factor # Spring constant
    c = damping ? spring.axial_damping : zero(spring.axial_damping) # Damping coefficient
    if (norm1 - l_0) > 0.0
        # kite springs are damped more
        c_t = i > segments ? s.set.rel_damping * c : c
        return k * (norm1 - l_0) + c_t * spring_vel
    elseif i > segments # kite spring
        return s.set.rel_compr_stiffness * k * (norm1 - l_0) + c * spring_vel
    else                # tether spring
        return 0.1 * k * (norm1 - l_0) + c * spring_vel
    end
end

"""
    calc_particle_forces!(s::KPS4, pos1, pos2, vel1, vel2, spring, segments, d_tether, rho, i)

Calculate the drag force of the tether segment, defined by the parameters pos1, pos2, vel1 and vel2
and distribute it equally on the two particles, that are attached to the segment.
The result is stored in the array s.forces.
"""
@inline function calc_particle_forces!(s::KPS4, pos1, pos2, vel1, vel2, spring, segments, d_tether, rho, i)
    segment = pos1 - pos2
    rel_vel = vel1 - vel2
    av_vel = 0.5 * (vel1 + vel2)
    norm1 = norm(segment)
    unit_vector = segment / norm1

    spring_vel   = unit_vector ⋅ rel_vel
    s.spring_force .= calc_spring_force(s, spring, norm1, spring_vel, segments, i) * unit_vector

    s.v_apparent .= s.v_wind_tether - av_vel
    half_drag_force, kcu_drag_force = calc_drag_forces(s, unit_vector, norm1, vel1, vel2,
                                                       SVec3(s.v_wind_tether), segments, d_tether, rho, i)
    if i == segments
        @inbounds s.forces[spring.p2] .+= kcu_drag_force
    end

    @inbounds s.forces[spring.p1] .+= half_drag_force + s.spring_force
    @inbounds s.forces[spring.p2] .+= half_drag_force - s.spring_force
    if i == 1 s.last_force .= s.forces[spring.p1] end
    nothing
end

"""
    calc_aero_forces(s::KPS4, pos, vel, rho, alpha_depower, rel_steering)

Calculates the aerodynamic forces acting on the kite particles without changing `s`.

Parameters:
- pos:              vector of the particle positions
- vel:              vector of the particle velocities
- rho:              air density [kg/m^3]
- alpha_depower:    depower angle [degrees]
- rel_steering:     value between -1.0 and +1.0

Returns a named tuple with the kite reference frame `x`, `y`, `z`, the `side_slip`, the angles
of attack `alphas` [degrees], the side lift coefficient `side_cl`, the lift forces `L2`, `L3`,
`L4`, the drag forces `D2`, `D3`, `D4`, the pitch damping and steering forces `f_d` and `f_s`
and the resulting forces on the kite particles `f_A`, `f_B`, `f_C` and `f_D`. Like `s.forces`,
all forces have the opposite sign of the physical forces.
"""
@inline function calc_aero_forces(s::KPS4, pos, vel, rho, alpha_depower, rel_steering)
    rel_side_area = Float64(s.set.rel_side_area)/100.0   # defined in percent
    K = 1 - rel_side_area                        # correction factor for the drag
    # pos_B, pos_C, pos_D: position of the kite particles B, C, and D
    # v_B,   v_C,   v_D:   velocity of the kite particles B, C, and D
    # immutable copies, so that no intermediate vector is allocated on the heap
    i_A = s.set.segments + 2
    pos_B, pos_C, pos_D = SVec3(pos[i_A+1]), SVec3(pos[i_A+2]), SVec3(pos[i_A+3])
    v_A, v_B, v_C, v_D  = SVec3(vel[i_A]), SVec3(vel[i_A+1]), SVec3(vel[i_A+2]), SVec3(vel[i_A+3])
    v_wind = SVec3(s.v_wind)
    va_1, va_2,  va_3,  va_4  = v_wind - v_A, v_wind - v_B, v_wind - v_C, v_wind - v_D

    pos_centre = 0.5 * (pos_C + pos_D)
    delta = pos_B - pos_centre
    z = -normalize(delta)
    y = normalize(pos_C - pos_D)
    x = y × z

    va_xz1 = va_1 - (va_1 ⋅ y) * y
    va_xz2 = va_2 - (va_2 ⋅ y) * y
    va_xy3 = va_3 - (va_3 ⋅ z) * z
    va_xy4 = va_4 - (va_4 ⋅ z) * z

    R_k_w = SMatrix{3,3}([x y z])
    va_k = R_k_w' * SVector{3}(va_2)
    side_slip = atan(va_k[2], -va_k[1])

    alpha_2 = rad2deg(π - acos2(normalize(va_xz2) ⋅ x) - alpha_depower)     + Float64(s.set.alpha_zero)
    alpha_3 = rad2deg(π - acos2(normalize(va_xy3) ⋅ x) + rel_steering * s.ks) + Float64(s.set.alpha_ztip)
    alpha_4 = rad2deg(π - acos2(normalize(va_xy4) ⋅ x) - rel_steering * s.ks) + Float64(s.set.alpha_ztip)
    alphas = SVec3(alpha_2, alpha_3, alpha_4)

    # Wrap alpha values to [-180, 180] to stay within the CL/CD spline data range
    if alpha_2 > 180.0; alpha_2 -= 360.0; elseif alpha_2 < -180.0; alpha_2 += 360.0; end
    if alpha_3 > 180.0; alpha_3 -= 360.0; elseif alpha_3 < -180.0; alpha_3 += 360.0; end
    if alpha_4 > 180.0; alpha_4 -= 360.0; elseif alpha_4 < -180.0; alpha_4 += 360.0; end

    if s.set.version == 3
        drag_corr = 1.0
    else
        drag_corr = DRAG_CORR
    end

    CL2, CD2 = s.calc_cl(alpha_2)::Float64, drag_corr * s.calc_cd(alpha_2)::Float64
    CL3, CD3 = s.calc_cl(alpha_3)::Float64, drag_corr * s.calc_cd(alpha_3)::Float64
    CL4, CD4 = s.calc_cl(alpha_4)::Float64, drag_corr * s.calc_cd(alpha_4)::Float64
    L2 = (-0.5 * rho * (norm(va_xz2))^2 * s.set.area * CL2) * normalize(va_2 × y)
    L3 = (-0.5 * rho * (norm(va_xy3))^2 * s.set.area * rel_side_area * CL3) * normalize(va_3 × z)
    L4 = (-0.5 * rho * (norm(va_xy4))^2 * s.set.area * rel_side_area * CL4) * normalize(z × va_4)
    D2 = (-0.5 * K * rho * norm(va_2) * s.set.area * CD2) * va_2
    D3 = (-0.5 * K * rho * norm(va_3) * s.set.area * rel_side_area * CD3) * va_3
    D4 = (-0.5 * K * rho * norm(va_4) * s.set.area * rel_side_area * CD4) * va_4
    f_d = (0.5 * rho * s.set.area * norm(va_xz1)^2 * s.set.cmq * s.pitch_rate * s.set.cord_length) * z
    f_s = (0.5 * rho * s.set.area * (0.5*(norm(va_xy3)+norm(va_xy4)))^2 * s.set.smc * rel_steering * s.ks) * x
    if s.set.version == 3
        f_B = L2 + D2 - D3 - D4
    else
        f_B = L2 + D2
    end
    f_A = f_d
    f_C = L3 + D3 - 0.5 * f_d + 0.5 * f_s
    f_D = L4 + D4 - 0.5 * f_d - 0.5 * f_s
    (; x, y, z, side_slip, alphas, side_cl = CL4 - CL3, L2, L3, L4, D2, D3, D4, f_d, f_s,
       f_A, f_B, f_C, f_D)
end

"""
    calc_aero_forces!(s::KPS4, pos, vel, rho, alpha_depower, rel_steering)

Calculates the aerodynamic forces acting on the kite particles with `calc_aero_forces`
and adds them to `s.forces`. Also stores the kite reference frame, the angles of attack and
the lift, drag and side forces in `s`.
"""
@inline function calc_aero_forces!(s::KPS4, pos, vel, rho, alpha_depower, rel_steering)
    a = calc_aero_forces(s, pos, vel, rho, alpha_depower, rel_steering)
    s.x .= a.x; s.y .= a.y; s.z .= a.z # save the kite reference frame in the state
    s.side_slip = a.side_slip
    s.alpha_2, s.alpha_3, s.alpha_4 = a.alphas
    s.side_cl = a.side_cl
    s.lift_force .= a.L2
    if s.set.version == 3
        s.drag_force .= a.D2
    else
        s.drag_force .= a.D2 + a.D3 + a.D4
    end
    s.f_d .= a.f_d
    s.f_s .= a.f_s
    s.forces[s.set.segments + 2] .+= a.f_A
    s.forces[s.set.segments + 3] .+= a.f_B
    s.forces[s.set.segments + 4] .+= a.f_C
    s.forces[s.set.segments + 5] .+= a.f_D
    s.side_force .= a.L3 + a.L4
end

"""
    aero_point_forces(s::KPS4)

Aerodynamic forces on the kite particles A, B, C and D in ENU, including the pitch damping
and steering couples, for the positions `s.pos` and velocities `s.vel`.

Returns a tuple `(f_A, f_B, f_C, f_D)` of four `SVec3` in [N].
"""
function aero_point_forces(s::KPS4)
    a = calc_aero_forces(s, s.pos, s.vel, s.rho, s.alpha_depower, s.steering)
    # the force vectors of KPS4 have the opposite sign of the physical forces
    -SVec3(a.f_A), -SVec3(a.f_B), -SVec3(a.f_C), -SVec3(a.f_D)
end

"""
    calc_aero_force_moment(s::KPS4)

Total aerodynamic force and moment acting on the kite, both resolved in the `KA` body
frame (aft-right-up). The moment is taken about the origin of the `KA` (and `KS`) frame, the
centre between the kite particles C and D, and includes the pitch damping and steering couples. Uses the positions
`s.pos`, the velocities `s.vel` and the kite reference frame of the last call of
`calc_aero_forces!`.

Returns a tuple `(force, moment)` of two `SVec3` in [N] and [Nm].
"""
function calc_aero_force_moment(s::KPS4)
    i_A = s.set.segments + 2
    pos_A, pos_B = SVec3(s.pos[i_A]),   SVec3(s.pos[i_A+1])
    pos_C, pos_D = SVec3(s.pos[i_A+2]), SVec3(s.pos[i_A+3])
    centre = 0.5 * (pos_C + pos_D) # origin of the KA and KS frames

    f_A, f_B, f_C, f_D = aero_point_forces(s)
    force  = f_A + f_B + f_C + f_D
    moment = (pos_A - centre) × f_A + (pos_B - centre) × f_B +
             (pos_C - centre) × f_C + (pos_D - centre) × f_D

    # project on the KS body axes (ENU), then flip to KA
    x, y, z = SVec3(s.x), SVec3(s.y), SVec3(s.z)
    force_KA  = fromKS2KA_body(SVec3(force ⋅ x,  force ⋅ y,  force ⋅ z))
    moment_KA = fromKS2KA_body(SVec3(moment ⋅ x, moment ⋅ y, moment ⋅ z))
    force_KA, moment_KA
end

"""
    calc_turn_rates(s::KPS4)

Angular velocity of the kite, resolved in the `KA` body frame (aft-right-up). It is calculated
from the positions `s.pos` and velocities `s.vel` of the kite particles B, C and D, which define
the kite reference frame.

Returns an `SVec3` in [rad/s].
"""
function calc_turn_rates(s::KPS4)
    i_B = s.set.segments + 3
    pos_B, pos_C, pos_D = SVec3(s.pos[i_B]), SVec3(s.pos[i_B+1]), SVec3(s.pos[i_B+2])
    vel_B, vel_C, vel_D = SVec3(s.vel[i_B]), SVec3(s.vel[i_B+1]), SVec3(s.vel[i_B+2])
    # time derivative of the unit vector a / norm(a)
    d_unit(a, da) = (da - (a ⋅ da) / (a ⋅ a) * a) / norm(a)
    # the kite reference frame as in calc_aero_forces and its time derivative
    span  = pos_C - pos_D
    delta = pos_B - 0.5 * (pos_C + pos_D)
    y, z  = normalize(span), -normalize(delta)
    x     = y × z
    yd    = d_unit(span, vel_C - vel_D)
    zd    = -d_unit(delta, vel_B - 0.5 * (vel_C + vel_D))
    xd    = yd × z + y × zd
    # angular velocity in ENU; exact for a rigid frame, a least squares fit otherwise
    ω = 0.5 * (x × xd + y × yd + z × zd)
    # project on the KS body axes, then flip to KA
    fromKS2KA_body(SVec3(ω ⋅ x, ω ⋅ y, ω ⋅ z))
end

"""
    inner_loop!(s::KPS4, pos, vel, v_wind_gnd, segments, d_tether)

Calculate the forces, acting on all particles.

Output:
- s.forces
- `s.v_wind_tether`
"""
@inline function inner_loop!(s::KPS4, pos, vel, v_wind_gnd, segments, d_tether)
    for i in eachindex(s.springs)
        p1 = s.springs[i].p1  # First point nr.
        p2 = s.springs[i].p2  # Second point nr.
        height = 0.5 * (pos[p1][3] + pos[p2][3])
        rho = calc_rho(s.am, height)
        # if height <= -1000.0
            # println("Error: height: $height")
        # end
        @assert height > -1000
        if height < 6
            height = 6
        end
        s.v_wind_tether .= calc_wind_factor(s.am, height) * v_wind_gnd
        calc_particle_forces!(s, pos[p1], pos[p2], vel[p1], vel[p2], s.springs[i], segments, d_tether, rho, i)
    end
    nothing
end

"""
    tether_drag_forces(s::KPS4)

Drag forces of the tether, the bridle lines and the KCU on each particle in ENU, for the
positions `s.pos` and velocities `s.vel`. The wind is calculated as in `inner_loop!`.

Returns an `SVector` with one `SVec3` per particle in [N].
"""
function tether_drag_forces(s::KPS4{S, T, P}) where {S, T, P}
    forces = zeros(MVector{P, SVec3})
    segments = s.set.segments
    d_tether = s.set.d_tether/1000.0
    for (i, spring) in pairs(s.springs)
        pos1, pos2 = SVec3(s.pos[spring.p1]), SVec3(s.pos[spring.p2])
        vel1, vel2 = SVec3(s.vel[spring.p1]), SVec3(s.vel[spring.p2])
        height = 0.5 * (pos1[3] + pos2[3])
        rho = calc_rho(s.am, height)
        height = max(height, 6)
        v_wind_tether = calc_wind_factor(s.am, height) * SVec3(s.v_wind_gnd)
        segment = pos1 - pos2
        norm1 = norm(segment)
        half_drag_force, kcu_drag_force = calc_drag_forces(s, segment / norm1, norm1, vel1, vel2,
                                                           v_wind_tether, segments, d_tether, rho, i)
        # the force vectors of KPS4 have the opposite sign of the physical forces
        forces[spring.p1] -= half_drag_force
        forces[spring.p2] -= half_drag_force + kcu_drag_force
    end
    SVector(forces)
end

"""
    loop!(s::KPS4, pos, vel, posd, veld)

Calculate the vectors s.res1 and calculate s.res2 using loops
that iterate over all tether segments.
"""
function loop!(s::KPS4, pos, vel, posd, veld)
    L_0      = s.l_tether / s.set.segments
    if s.set.version == 1
        # for compatibility with the python code and paper
        mass_per_meter = 0.011
    else
        mass_per_meter = s.set.rho_tether * π * (s.set.d_tether/2000.0)^2
    end
    s.res1[1] .= pos[1]
    s.res2[1] .= vel[1]
    particles = s.set.segments + KITE_PARTICLES + 1
    for i in 2:particles
        s.res1[i] .= vel[i] - posd[i]
    end
    # Compute the masses and forces
    m_tether_particle = mass_per_meter * s.segment_length
    s.masses[s.set.segments+1] = s.set.kcu_mass + 0.5 * m_tether_particle
    # TODO: check if the next two lines are correct
    axial_damping  = s.set.axial_damping / L_0
    axial_stiffness = s.set.axial_stiffness/L_0
    for i in 1:s.set.segments
        @inbounds s.masses[i] = m_tether_particle
        @inbounds s.springs[i] = SP(s.springs[i].p1, s.springs[i].p2, s.segment_length, axial_stiffness, axial_damping)
    end
    inner_loop!(s, pos, vel, s.v_wind_gnd, s.set.segments, s.set.d_tether/1000.0)
    for i in 2:particles
        s.res2[i] .= veld[i] - (SVector(0, 0, -G_EARTH) - s.forces[i] / s.masses[i])
    end
end

"""
    residual!(res, yd, y::MVector{S, SimFloat}, s::KPS4, time) where S

    N-point tether model, four points for the kite on top:
    Inputs:
    State vector y   = pos1,  pos2, ... , posn,  vel1,  vel2, . .., veln,  length, v_reel_out
    Derivative   yd  = posd1, posd2, ..., posdn, veld1, veld2, ..., veldn, lengthd, v_reel_outd
    Output:
    Residual     res = res1, res2 = vel1-posd1,  ..., veld1-acc1, ...,

    Additional parameters:
    s: Struct with work variables, type KPS4
    S: The dimension of the state vector
The number of the point masses of the model N = S/6, the state of each point
is represented by two 3 element vectors.
"""
function residual!(res, yd, y::Vector{SimFloat}, s::KPS4, time)
    S = length(y)
    y_ =  MVector{S, SimFloat}(y)
    yd_ =  MVector{S, SimFloat}(yd)
    residual!(res, yd_, y_, s, time)
end
function residual!(res, yd, y::MVector{S, SimFloat}, s::KPS4, _) where S
    T = S-2 # T: three times the number of particles excluding the origin
    segments = div(T,6) - KITE_PARTICLES

    # Reset the force vector to zero.
    for i in 1:segments+KITE_PARTICLES+1
        s.forces[i] .= SVector(0.0, 0, 0)
    end
    # extract the data for the winch simulation
    length,  v_reel_out  = y[end-1],  y[end]
    lengthd, v_reel_outd = yd[end-1], yd[end]
    # extract the data of the particles
    y_  = @view y[1:end-2]
    yd_ = @view yd[1:end-2]
    # unpack the vectors y and yd
    part  = reshape(SVector{T}(y_),  Size(3, div(T,6), 2))
    partd = reshape(SVector{T}(yd_), Size(3, div(T,6), 2))
    pos1, vel1 = part[:,:,1], part[:,:,2]
    pos = SVector{div(T,6)+1}(if i==1 SVector(0.0,0,0) else SVector(pos1[:,i-1]) end for i in 1:div(T,6)+1)
    vel = SVector{div(T,6)+1}(if i==1 SVector(0.0,0,0) else SVector(vel1[:,i-1]) end for i in 1:div(T,6)+1)
    posd1, veld1 = partd[:,:,1], partd[:,:,2]
    posd = SVector{div(T,6)+1}(if i==1 SVector(0.0,0,0) else SVector(posd1[:,i-1]) end for i in 1:div(T,6)+1)
    veld = SVector{div(T,6)+1}(if i==1 SVector(0.0,0,0) else SVector(veld1[:,i-1]) end for i in 1:div(T,6)+1)
    @assert isfinite(pos[2][3])

    # core calculations
    s.l_tether = length
    s.segment_length = length / segments
    calc_aero_forces!(s, pos, vel, s.rho, s.alpha_depower, s.steering)
    loop!(s, pos, vel, posd, veld)

    # winch calculations
    res[end-1] = lengthd - v_reel_out
    # the state of the winch (brake, rate limited set speed) is updated once per time step in next_step!
    if s.wm isa AsyncMachine
        res[end] = v_reel_outd - calc_acceleration(s.wm::AsyncMachine, v_reel_out, norm(s.forces[1]);
            set_speed=s.sync_speed, use_brake=true, update_state=false)
    elseif !isnothing(s.wm)
        res[end] = v_reel_outd - calc_acceleration(s.wm, v_reel_out, norm(s.forces[1]); set_speed=s.sync_speed,
            set_torque=s.set_torque, use_brake=use_brake(s), update_state=false)
    else
        res[end] = v_reel_outd
    end
    # copy and flatten result
    for i in 2:div(T,6)+1
        for j in 1:3
            @inbounds res[3*(i-2)+j]              = s.res1[i][j]
            @inbounds res[3*(div(T,6))+3*(i-2)+j] = s.res2[i][j]
        end
    end
    # copy the position and velocity vectors for easy debugging
    for i in 1:div(T,6)+1
        @inbounds s.pos[i] .= pos[i]
        @inbounds s.vel[i] .= vel[i]
    end
    s.vel_kite .= vel[end-2]
    s.v_reel_out = v_reel_out

    @assert ! isnan(norm(res))
    s.iter += 1

    nothing
end

# =================== getter functions ====================================================

"""
    calc_height(s::KPS4)

Determine the height of the topmost kite particle above ground.
"""
function calc_height(s::KPS4)
    pos_kite(s)[3]
end

"""
    pos_kite(s::KPS4)

Return the position of the kite (top particle).
"""
function pos_kite(s::KPS4)
    s.pos[end-2]
end

"""
    kite_ref_frame(s::KPS4; one_point=false)

Returns a tuple of the x, y, and z vectors of the kite reference frame.
"""
function kite_ref_frame(s::KPS4; one_point=false)
    if one_point
        c = s.z
        y = normalize(s.v_apparent × c)
        x = normalize(y × c)
        return x, y, c
    else
        return s.x, s.y, s.z
    end
end

"""
    winch_force(s::KPS4)

Return the absolute value of the force at the winch as calculated during the last timestep.
"""
function winch_force(s::KPS4) norm(s.last_force) end

"""
    reel_out_speed(s::KPS4)

Return the current reel-out speed of the winch in m/s.
"""
function reel_out_speed(s::KPS4) s.v_reel_out end

"""
    cl_cd(s::KPS4)

Calculate the lift and drag coefficients of the kite, based on the current angles of attack.
"""
function cl_cd(s::KPS4)
    rel_side_area = Float64(s.set.rel_side_area)/100.0 # defined in percent
    K = 1 - rel_side_area                      # correction factor for the drag
    if s.set.version == 3
        drag_corr = 1.0
    else
        drag_corr = DRAG_CORR
    end
    a2, a3, a4 = s.alpha_2, s.alpha_3, s.alpha_4
    if a2 > 180.0; a2 -= 360.0; elseif a2 < -180.0; a2 += 360.0; end
    if a3 > 180.0; a3 -= 360.0; elseif a3 < -180.0; a3 += 360.0; end
    if a4 > 180.0; a4 -= 360.0; elseif a4 < -180.0; a4 += 360.0; end
    CL2, CD2 = s.calc_cl(a2), drag_corr * s.calc_cd(a2)
    _, CD3   = s.calc_cl(a3), drag_corr * s.calc_cd(a3)
    _, CD4   = s.calc_cl(a4), drag_corr * s.calc_cd(a4)
    if s.set.version == 3
        return CL2, CD2
    else
        return CL2, K*(CD2+CD3+CD4)
    end
end

# ==================== end of getter functions ================================================

"""
    spring_forces(s::KPS4; prn=true, damping=true)

Return the scalar forces of all springs [N], positive for tension: first the tether segments,
then the springs of the kite. They are the forces of the residual for the positions `s.pos`
and velocities `s.vel`, including the damping force, unless `damping=false`. With `prn=true`,
a warning is printed if a force exceeds the breaking force.
"""
function spring_forces(s::KPS4; prn=true, damping=true)
    segments = s.set.segments
    forces = zeros(SimFloat, segments+KITE_SPRINGS)
    for (i, spring) in pairs(s.springs)
        segment = s.pos[spring.p1] - s.pos[spring.p2]
        norm1 = norm(segment)
        spring_vel = (segment / norm1) ⋅ (s.vel[spring.p1] - s.vel[spring.p2])
        forces[i] = calc_spring_force(s, spring, norm1, spring_vel, segments, i; damping)
        if prn && i <= segments && forces[i] > s.set.max_force
            println("Tether raptures for segment $i !")
        elseif prn && i > segments && forces[i] > 4000.0
            println("Bridle brakes for spring $(i - segments) !")
        end
    end
    forces
end

function turn(res, upwind_dir)
    turnangle = upwind_dir + pi/2
    res2 = zeros(SimFloat, length(res))
    for i in 1:div(length(res), 3)
        x = res[3*(i-1)+1]
        y = res[3*(i-1)+2]
        res2[3*(i-1)+1] = cos(turnangle) * x + sin(turnangle) * y
        res2[3*(i-1)+2] = cos(turnangle) * y - sin(turnangle) * x
        res2[3*(i-1)+3] = res[3*(i-1)+3]
    end
    res2
end

"""
    find_steady_state!(s::KPS4; prn=false, upwind_dir=-pi/2, trim=false)

Find an initial equilibrium, based on the initial parameters
`l_tether`, elevation and `v_reel_out`.

The forces on the tether particles, the KCU and the kite particles are balanced (without
gravity), except for the z component of the kite points C and D: instead, the elevation
of the kite (`pos_kite`, point B) is kept at `s.set.elevation`, because the kite can only
be fully balanced at its natural elevation. Therefore `calc_elevation(s)` returns
`s.set.elevation` after the steady state was found. If the prescribed elevation is far
below the natural elevation, the kite can only be balanced in deep stall, with an angle of
attack beyond the angle of maximum lift; then a warning is logged.

If `trim` is `true`, all forces including gravity are balanced, also the z components of
the forces on C and D, and the elevation is solved for instead: the kite is placed at the
elevation where its lift matches gravity (its natural elevation), starting the search at
80° or at `s.set.elevation`, if it is higher: the solver converges from above the natural
elevation, but often not from far below it.
Trimmed steady states with an angle of attack beyond the angle of maximum lift (deep
stall) are rejected. If no valid trim is found from the start elevation, the solver is
restarted at higher and then at lower elevations than `s.set.elevation`, in steps of 10°.
If this fails too, an error is thrown, and `init!` returns `nothing`.

The steady state is always solved for the full tether stiffness (`s.stiffness_factor` is 1.0
afterwards). The nonlinear solver is sensitive to the starting point, so small floating-point
differences (e.g. between Julia versions) can decide whether it converges. If the
first attempt fails, it is repeated without autoscaling and then by continuation:
a solution is first found for a reduced tether stiffness and then used as start value
for the full stiffness. The initial state is not perturbed, unless this fails; the returned
steady state is never perturbed.

The keyword arguments `delta` and `stiffness_factor` are deprecated and ignored.
"""
function find_steady_state!(s::KPS4; prn=false, delta=nothing, stiffness_factor=nothing, upwind_dir=-pi/2,
                            trim=false)
    warn_delta_deprecated(delta, :find_steady_state!)
    warn_stiffness_factor_deprecated(stiffness_factor, :find_steady_state!)
    # solve without perturbation of the initial state, with a perturbation only as fallback; the
    # restarts at other elevations use a cheaper solve, so that failing to find a trim is fast
    function solve(; restart=false)
        r, conv = nothing, false
        for delta in (restart ? (0.0,) : STEADY_STATE_DELTAS)
            r, conv = solve_steady_state!(s; prn, delta, trim, continuation=!restart)
            conv && break
        end
        r, conv
    end
    el_set = s.set.elevation
    # elevation of the first attempt; the offsets of its result are relative to it
    el_start = trim ? max(el_set, TRIM_START_ELEVATION) : el_set
    alpha_stall = s.set.alpha_cl[argmax(s.set.cl_list)]  # angle of attack of maximum lift [°]
    res = zeros(MVector{6*(s.set.segments+KITE_PARTICLES)+2, SimFloat})
    # the state for the offsets X, which are relative to the tether at s.set.elevation
    function steady_state(X)
        y_, yd_ = init(s, X; upwind_dir)
        # the accelerations of a steady state that is balanced with gravity are zero
        if trim
            n = 3*(s.set.segments+KITE_PARTICLES)
            yd_[n+1:2n] .= 0.0
        end
        set_v_wind_ground!(s, y_[3*s.set.segments+6], s.set.v_wind; upwind_dir)
        residual!(res, yd_, y_, s, 0.0)
        y_, yd_
    end
    y00, yd00 = nothing, nothing
    try
        s.set.elevation = el_start
        results, converged = solve()
        converged && ((y00, yd00) = steady_state(results.zero))
        stalled = trim && converged && (s.alpha_2 > alpha_stall || !above_horizon(s))
        if trim && (stalled || !converged)
            # Restart from other elevations: the solver converges from above the natural
            # elevation, but diverges or finds a deep stall trim from far below it.
            starts = [el_set .+ (TRIM_RESTART_STEP:TRIM_RESTART_STEP:85.0);
                      el_set .- (TRIM_RESTART_STEP:TRIM_RESTART_STEP:85.0)]
            for el in filter(el -> 5.0 <= el <= 89.0 && el != el_start, starts)
                s.set.elevation = el
                r, conv = solve(restart=true)
                conv || continue
                y, yd = steady_state(r.zero)
                if s.alpha_2 <= alpha_stall && above_horizon(s)
                    prn && println("find_steady_state!: trimmed steady state found, starting at $(el)°")
                    results, converged, stalled = r, true, false
                    y00, yd00 = y, yd
                    break
                end
            end
            # no valid trim found: a deep stall or non-converged state is not a valid initial state
            if stalled || !converged
                error("find_steady_state!: no trimmed steady state with attached flow was found; " *
                      "the solver did not converge, or the kite can only be balanced in deep stall")
            end
        end
        if !converged
            s.set.elevation = el_start
            @warn "find_steady_state!: solver did not converge! (f_converged=$(results.f_converged), x_converged=$(results.x_converged), iterations=$(results.iterations), residual_norm=$(results.residual_norm))"
            # Check if the solution contains finite values
            if !all(isfinite, results.zero)
                error("find_steady_state!: solver returned non-finite values. Cannot compute steady state.")
            end
            y00, yd00 = steady_state(results.zero)
        end
        if !trim && converged && s.alpha_2 > alpha_stall
            @warn "find_steady_state!: the kite is in deep stall at the prescribed elevation of " *
                  "$(el_set)° (angle of attack $(round(s.alpha_2, digits=1))°); use trim=true or a " *
                  "higher elevation"
        end
        return y00, yd00
    catch e
        if e isa AssertionError
            error("find_steady_state!: computation resulted in non-finite values. The solver did not find a valid steady state.")
        else
            rethrow(e)
        end
    finally
        s.set.elevation = el_set
    end
end

# Stiffness factors for the continuation in solve_steady_state!: if the solve for the full tether
# stiffness fails, the solution is first found for the reduced stiffness factors of a schedule,
# each starting from the previous solution. A benchmark with system.yaml (wind speeds of 6 to
# 25 m/s, elevations of 40° to 85°, tether lengths of 50 to 1000 m, with and without trim) showed
# that the solve for the full stiffness converges directly in most cases and is the fastest
# option, starting at a reduced stiffness is needed mainly with trim, and the schedules fail in
# different cases, so they are tried in turn.
const STIFFNESS_SCHEDULES = ((0.1, 1.0), (0.1, 0.3, 1.0), (0.3, 1.0))

# Solve for the offsets of the steady state for the full tether stiffness, see
# find_steady_state!. Returns the result of nlsolve and whether it is usable.
function solve_steady_state!(s::KPS4; prn=false, delta=0.001, trim=false, continuation=true)
    stiffness_factor = 1.0
    s.stiffness_factor = stiffness_factor
    res = zeros(MVector{6*(s.set.segments+KITE_PARTICLES)+2, SimFloat})
    delta_ref = Ref(delta)

    # helper function for the steady state finder
    function test_initial_condition!(F, x::Vector)
        x1 = copy(x)
        y0, yd0 = init(s, x1; delta=delta_ref[])
        # init sets the vertical accelerations to -g, so that gravity is not balanced; with trim
        # the accelerations are zero, so that the kite and the tether are balanced with gravity
        if trim
            n = 3*(s.set.segments+KITE_PARTICLES)
            yd0[n+1:2n] .= 0.0
        end
        # the wind at the height of the kite (z of point B) of this candidate state
        set_v_wind_ground!(s, y0[3*s.set.segments+6], s.set.v_wind; upwind_dir=-pi/2)
        try
            residual!(res, yd0, y0, s, 0.0)
        catch e
            @debug "Infeasible point in test_initial_condition!" exception=e
            # Fill F with large values so the solver treats this point as infeasible
            F .= 1.0e6
            return nothing
        end
        # The unknowns x are the x and z offsets of the particles 2..segments+1 (tether and KCU),
        # p8, p9 and C, an unused entry and the y position of C; D is the mirror image of C.
        # One equation per unknown: the x and z residuals of the accelerations of the particles
        # 2..segments+1, p8 and p9, the symmetric part of the x residual of C and D, the elevation
        # of the kite, the unused entry and the antisymmetric part of the y residual of C and D.
        # The kite can only be fully balanced at its natural elevation, therefore the elevation
        # of the kite is prescribed instead of the z residual of C and D, unless trim is true.
        segments = s.set.segments
        n_free = segments + KITE_PARTICLES - 1            # number of x (and of z) unknowns
        acc(i, k) = res[k + 3*(i-2) + 3*(segments+KITE_PARTICLES)]  # residual of particle i, component k
        for (n, i) in enumerate(2:segments+3)
            F[n]          = acc(i, 1)
            F[n + n_free] = acc(i, 3)
        end
        C, D = segments+4, segments+5
        F[n_free]   = 0.5 * (acc(C, 1) + acc(D, 1))
        # position of the kite as returned by pos_kite (point B = p9 = particle segments+3)
        if trim
            F[2*n_free] = 0.5 * (acc(C, 3) + acc(D, 3))
        else
            kite = SVector(y0[3*segments+4], y0[3*segments+5], y0[3*segments+6])
            F[2*n_free] = calc_elevation(kite) - deg2rad(s.set.elevation)
        end
        F[end-1]    = x[end-1]
        F[end]      = 0.5 * (acc(C, 2) - acc(D, 2))
        return nothing
    end
    if prn println("\nStarted function test_nlsolve...") end
    X00 = zeros(SimFloat, 2*(s.set.segments+KITE_PARTICLES-1)+2)
    jac! = make_jac(test_initial_condition!, length(X00))
    function solve(sf, d, x0; autoscale=true)
        s.stiffness_factor = sf
        delta_ref[] = d
        nlsolve(test_initial_condition!, jac!, copy(x0); autoscale, xtol=4e-7, ftol=4e-7,
                iterations=s.set.max_iter)
    end
    # accept a result only if it is finite and the residual is small
    usable(r) = all(isfinite, r.zero) && (r.f_converged || (r.x_converged && r.residual_norm < 1e-3))
    # returns the result for (sf, d), trying both scaling options, or nothing
    function try_solve(sf, d, x0)
        for autoscale in (true, false)
            local r = solve(sf, d, x0; autoscale)
            usable(r) && return r
        end
        nothing
    end
    results = solve(stiffness_factor, delta, X00)
    if !usable(results)
        r = solve(stiffness_factor, delta, X00; autoscale=false)
        usable(r) || (r = nothing)
        # continuation with a fixed schedule of stiffness factors
        for schedule in STIFFNESS_SCHEDULES
            isnothing(r) || break
            r = try_solve(schedule[1], delta, X00)
            for sf in schedule[2:end]
                isnothing(r) && break
                r = try_solve(sf, delta, r.zero)
            end
            !isnothing(r) && prn && println("find_steady_state!: converged with the stiffness schedule $schedule")
        end
        if isnothing(r) && continuation
            # continuation: solve an easier problem, then step towards the full stiffness
            starts = [(sf0, delta) for sf0 in (0.1, 0.2, 0.3, 0.05) if sf0 != stiffness_factor]
            if delta != 0.001
                push!(starts, (stiffness_factor, 0.001))
                append!(starts, [(sf0, 0.001) for sf0 in (0.1, 0.2, 0.3, 0.05) if sf0 != stiffness_factor])
            end
            for (sf0, d0) in starts
                r = try_solve(sf0, d0, X00)
                for t in range(0, 1; length=5)[2:end]
                    isnothing(r) && break
                    r = try_solve(sf0 + t*(stiffness_factor-sf0), d0 + t*(delta-d0), r.zero)
                end
                if !isnothing(r)
                    prn && println("find_steady_state!: converged by continuation from stiffness_factor=$sf0, delta=$d0")
                    break
                end
            end
        end
        isnothing(r) || (results = r)
    end
    s.stiffness_factor = stiffness_factor
    delta_ref[] = delta
    if prn println("\nresult: $results") end
    results, usable(results)
end

const KITE_ANGLE = 3.83 # angle between the kite and the last tether segment due to the mass of the control pod
const PRE_STRESS  = 0.9998   # Multiplier for the initial spring lengths.

# Functions to calculate the initial state vector, the initial masses and initial springs

function init_springs!(s::KPS4)
    l_0     = s.set.l_tethers[1] / s.set.segments
    particles = KiteUtils.get_particles(s.set.height_k, s.set.h_bridle, s.set.width, s.set.m_k)
    for j in 1:size(SPRINGS_INPUT)[1]
        # build the tether segments
        if j == 1
            for i in 1:s.set.segments
                k = s.set.e_tether * (s.set.d_tether/2000.0)^2 * pi  / l_0  # Spring stiffness for this spring [N/m]
                c = s.set.axial_damping/l_0                                       # Damping coefficient [Ns/m]
                s.springs[i] = SP(i, i+1, l_0, k, c)
            end
        # build the bridle segments
        else
            p0, p1 = SPRINGS_INPUT[j, 1]+1, SPRINGS_INPUT[j, 2]+1 # point 0 and 1
            if SPRINGS_INPUT[j, 3] == -1
                l_0 = norm(particles[Int(p1)] - particles[Int(p0)]) * PRE_STRESS
                k = s.set.e_tether * (s.set.d_line/2000.0)^2 * pi / l_0
                p0 += s.set.segments - 1 # correct the index for the start and end particles of the bridle
                p1 += s.set.segments - 1
                c = s.set.axial_damping/ l_0
                s.springs[j+s.set.segments-1] = SP(Int(p0), Int(p1), l_0, k, c)
            end
        end
    end
    s.springs
end

function init_masses!(s::KPS4)
    MASS_FACTOR = 1.0
    s.masses = zeros(s.set.segments+KITE_PARTICLES+1)
    l_0 = s.set.l_tethers[1] / s.set.segments
    for i in 1:s.set.segments
        s.masses[i]   += 0.5 * l_0 * s.set.rho_tether * (s.set.d_tether/2000.0)^2 * pi
        s.masses[i+1] += 0.5 * l_0 * s.set.rho_tether * (s.set.d_tether/2000.0)^2 * pi
    end
    s.masses[s.set.segments+1] += s.set.kcu_mass * MASS_FACTOR
    k2 = s.set.rel_top_mass * (1.0 - s.set.rel_nose_mass)
    k3 = 0.5 * (1.0 - s.set.rel_top_mass) * (1.0 - s.set.rel_nose_mass)
    k4 = 0.5 * (1.0 - s.set.rel_top_mass) * (1.0 - s.set.rel_nose_mass)
    s.masses[s.set.segments+2] += s.set.rel_nose_mass * s.set.mass * MASS_FACTOR
    s.masses[s.set.segments+3] += k2 * s.set.mass * MASS_FACTOR
    s.masses[s.set.segments+4] += k3 * s.set.mass * MASS_FACTOR
    s.masses[s.set.segments+5] += k4 * s.set.mass * MASS_FACTOR
    s.masses
end

function init_pos_vel_acc(s::KPS4, X=zeros(2 * (s.set.segments+KITE_PARTICLES)+1); old=false, delta = 0.0)
    pos = zeros(SVector{s.set.segments+1+KITE_PARTICLES, KVec3})
    vel = zeros(MVector{s.set.segments+1+KITE_PARTICLES, KVec3})
    acc = zeros(SVector{s.set.segments+1+KITE_PARTICLES, KVec3})
    pos[1] .= [0.0, delta, 0.0]
    vel[1] .= [delta, delta, delta]
    acc[1] .= [delta, delta, delta]
    sin_el, cos_el = sin(s.set.elevation::Float64 / 180.0 * pi), cos(s.set.elevation::Float64 / 180.0 * pi)
    for i in 1:s.set.segments
        radius = -i * (s.set.l_tethers[1]/s.set.segments)
        pos[i+1] .= [-cos_el * radius + X[i], delta, -sin_el * radius + X[s.set.segments+KITE_PARTICLES-1+i]]
        vel[i+1] .= [delta, delta, 0]
        acc[i+1] .= [delta, delta, -9.81]
    end
    vec_c = pos[s.set.segments] - pos[s.set.segments+1]
    if old
        particles = KiteUtils.get_particles(s.set.height_k, s.set.h_bridle, s.set.width, s.set.m_k)
    else
        particles = KiteUtils.get_particles(s.set.height_k, s.set.h_bridle, s.set.width, s.set.m_k,
                              pos[s.set.segments+1], rotate_around_y(vec_c, deg2rad(KITE_ANGLE)), s.v_apparent)
    end
    j = 1
    for i in [1,2,3] # set p8, p9, p10
        pos[s.set.segments+1+i] .= particles[i+2] + [X[s.set.segments+j], 0, X[2*s.set.segments+KITE_PARTICLES-1+j]]
        vel[s.set.segments+1+i] .= [delta, delta, delta]
        acc[s.set.segments+1+i] .= [delta, delta, -9.81]
        j +=1
    end
    acc[s.set.segments+1+4] .= [delta, delta, -9.81]
    vel[s.set.segments+1+4] .= [delta, delta, delta]
    # set p10=C and p11=D
    # x and z component of the right and left particle must be equal
    pos[s.set.segments+1+4][1] = pos[s.set.segments+1+3][1]  # D.x = C.x
    pos[s.set.segments+1+4][3] = pos[s.set.segments+1+3][3]  # D.z = C.z
    pos[s.set.segments+1+3][2] += X[end]                     # Y position of point C
    pos[s.set.segments+1+4][2] = -pos[s.set.segments+1+3][2] # Y position of point D
    for i in eachindex(pos)
        s.pos[i] .= pos[i]
    end
    for i in 2:s.set.segments+1
        vel[i] .+= normalize(pos[i] - pos[i-1]) * (s.set.v_reel_outs[1]*(i-1)/s.set.segments)
    end
    # the velocity vector of the kite particles is the same as the velocity of the last tether point
    for i in s.set.segments+2:s.set.segments+KITE_PARTICLES+1
        vel[i] .+= vel[s.set.segments+1]
    end
    pos, vel, acc
end

function set_initial_velocity!(s::KPS4)
    for i in 2:s.set.segments+1
        s.vel[i] .= normalize(s.pos[i] - s.pos[i-1]) * (s.set.v_reel_outs[1]*(i-1)/s.set.segments)
    end
    # the velocity vector of the kite particles is the same as the velocity of the last tether point
    for i in s.set.segments+2:s.set.segments+KITE_PARTICLES+1
        s.vel[i] .= s.vel[s.set.segments+1]
    end
end

# Calculate the initial vectors pos and vel. Tether with the initial elevation angle
# se().elevation, particle zero fixed at origin.
# X is a vector of deviations in x and z positions, to be varied to find the initial equilibrium
function init_pos_vel(s::KPS4, X=zeros(2 * (s.set.segments+KITE_PARTICLES)))
    pos, vel, _ = init_pos_vel_acc(s, X; old=true)
    pos, vel
end

function init_inner(s::KPS4, X=zeros(2 * (s.set.segments+KITE_PARTICLES-1)+1); old=false, delta=0.0)
    pos, vel, acc = init_pos_vel_acc(s, X; old=old, delta=delta)
    vcat(pos[2:end], vel[2:end]), vcat(vel[2:end], acc[2:end])
end

# same as above, but returns a tuple of two one dimensional arrays
function init(s::KPS4, X=zeros(2 * (s.set.segments+KITE_PARTICLES-1)+1); old=false, delta=0.0, upwind_dir=nothing)
    pos, vel, acc = init_pos_vel_acc(s, X; old=old, delta=delta)
    res1_ = vcat(pos[2:end], vel[2:end])
    res2_ = vcat(vel[2:end], acc[2:end])
    res1__ = reduce(vcat, res1_)
    res2__ = reduce(vcat, res2_)
    set_initial_velocity!(s)
    for i in 1:Int(length(res1__)/6)
        j = i + s.set.segments+KITE_PARTICLES
        # println("i, j:", i, ", ", j)
        res1__[3*(j-1)+1:3*j] .= s.vel[i+1]
        res2__[3*(i-1)+1:3*i] .= s.vel[i+1]
    end
    if !isnothing(upwind_dir)
        res1__ = turn(res1__, upwind_dir)
        res2__ = turn(res2__, upwind_dir)
    end
    res1, res2  = vcat(res1__, [s.l_tether, s.set.v_reel_out]),  vcat(res2__,[0,0])
    MVector{6*(s.set.segments+KITE_PARTICLES)+2, SimFloat}(res1), MVector{6*(s.set.segments+KITE_PARTICLES)+2, SimFloat}(res2)
end

# rotate a 3d vector around the x axis in the yz plane - following the right hand rule
function rotate_around_x(vec, angle::T) where T
    result = zeros(T, 3)
    result[1] = vec[1]
    result[2] = cos(angle) * vec[2] - sin(angle) * vec[3]
    result[3] = sin(angle) * vec[2] + cos(angle) * vec[3]
    result
end

# rotate a 3d vector around the y axis in the xz plane - following the right hand rule
function rotate_around_y(vec, angle::T) where T
    result = zeros(T, 3)
    result[1] = cos(angle) * vec[1] + sin(angle) * vec[3]
    result[2] = vec[2]
    result[3] = -sin(angle) * vec[1] + cos(angle) * vec[3]
    result
end

# rotate a 3d vector around the z axis in the yx plane - following the right hand rule
function rotate_around_z(vec, angle::T) where T
    result = zeros(T, 3)
    result[1] = cos(angle) * vec[1] - sin(angle) * vec[2]
    result[2] = sin(angle) * vec[1] + cos(angle) * vec[2]
    result[3] = vec[3]
    result
end
