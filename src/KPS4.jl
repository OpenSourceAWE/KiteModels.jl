# Copyright (c) 2020, 2021, 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

#= Model of a kite-power system in implicit form: residual = f(y, yd)

This model implements a 3D mass-spring system with reel-out. It uses six tether segments (the number can be
configured in the file data/settings.yaml). The kite is modelled using 4 point masses and 3 aerodynamic 
surfaces. The spring constant and the damping decrease with the segment length. The aerodynamic kite forces
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

# struct, defining the phyical parameters of one spring
@with_kw struct Spring{I, S}
    p1::I = 1         # number of the first point
    p2::I = 2         # number of the second point
    length::S = 1.0   # current unstressed spring length
    c_spring::S = 1.0 # spring constant [N/m]
    damping::S  = 0.1 # damping coefficent [Ns/m]
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
    wm::AbstractWinchModel
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
    "multiplier for the stiffniss of tether and bridle"
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
    s.v_reel_out = s.set.v_reel_out
    s.last_v_reel_out = s.set.v_reel_out
    s.sync_speed = s.set.v_reel_out
    s.v_wind_gnd    .= [s.set.v_wind, 0, 0]    # wind vector at reference height
    s.v_wind_tether .= [s.set.v_wind, 0, 0]
    s.v_apparent    .= [s.set.v_wind, 0, 0]
    height = sin(deg2rad(s.set.elevation)) * (s.set.l_tether)
    s.v_wind .= s.v_wind_gnd * calc_wind_factor(s.am, height)

    s.l_tether = s.set.l_tether
    s.segment_length = s.l_tether / s.set.segments
    init_masses!(s)
    init_springs!(s)
    for i in 1:s.set.segments + KiteModels.KITE_PARTICLES + 1 
        s.forces[i] .= zeros(3)
    end
    s.drag_force .= [0.0, 0, 0]
    s.lift_force .= [0.0, 0, 0]
    s.side_force .= [0.0, 0, 0]
    s.rho = s.set.rho_0
    s.bridle_factor = s.set.l_bridle / bridle_length(s.set)
    s.ks = deg2rad(s.set.max_steering) 
    s.kcu.depower = s.set.depower/100.0
    s.kcu.set_depower = s.kcu.depower
    roll, pitch, yaw = orient_euler(s)
    s.pitch = pitch
    s.pitch_rate = 0.0
    KiteModels.set_depower_steering!(s, get_depower(s.kcu), get_steering(s.kcu))
end

function KPS4(kcu::KCU)
    if kcu.set.winch_model == "AsyncMachine"
        wm = AsyncMachine(kcu.set)
    elseif kcu.set.winch_model == "TorqueControlledMachine"
        wm = TorqueControlledMachine(kcu.set)
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
    calc_particle_forces!(s::KPS4, pos1, pos2, vel1, vel2, spring, segments, d_tether, rho, i)

Calculate the drag force of the tether segment, defined by the parameters pos1, pos2, vel1 and vel2
and distribute it equally on the two particles, that are attached to the segment.
The result is stored in the array s.forces. 
"""
@inline function calc_particle_forces!(s::KPS4, pos1, pos2, vel1, vel2, spring, segments, d_tether, rho, i)
    l_0 = spring.length # Unstressed length
    k = spring.c_spring * s.stiffness_factor  # Spring constant
    c = spring.damping                        # Damping coefficient    
    segment = pos1 - pos2
    rel_vel = vel1 - vel2
    av_vel = 0.5 * (vel1 + vel2)
    norm1 = norm(segment)
    unit_vector = segment / norm1

    k1 = s.set.rel_compr_stiffness * k # compression stiffness kite springs
    k2 = 0.1 * k                       # compression stiffness tether springs
    c1 = s.set.rel_damping * c         # damping kite springs
    spring_vel   = unit_vector ⋅ rel_vel
    if (norm1 - l_0) > 0.0
        if i > segments  # kite springs
             s.spring_force .= (k *  (norm1 - l_0) + (c1 * spring_vel)) * unit_vector 
        else
             s.spring_force .= (k *  (norm1 - l_0) + (c * spring_vel)) * unit_vector
        end
    elseif i > segments # kite spring
        s.spring_force .= (k1 *  (norm1 - l_0) + (c * spring_vel)) * unit_vector
    else
        s.spring_force .= (k2 *  (norm1 - l_0) + (c * spring_vel)) * unit_vector
    end

    s.v_apparent .= s.v_wind_tether - av_vel
    v_app_kcu = s.v_wind_tether - vel2
    if s.set.version == 1
        area = norm1 * d_tether
    else
        if i > segments
            area = norm1 * s.set.d_line * 0.001 * s.bridle_factor # 6.0 = A_real/A_simulated
        else
            area = norm1 * d_tether
        end
    end

    v_app_perp = s.v_apparent - s.v_apparent ⋅ unit_vector * unit_vector
    half_drag_force = (-0.25 * rho * s.set.cd_tether * norm(v_app_perp) * area) * v_app_perp 
    if i == segments
        v_app_perp_kcu = v_app_kcu - v_app_kcu ⋅ unit_vector * unit_vector
        kcu_area = π * (s.set.kcu_diameter/2)^2
        kcu_drag_force = (-0.25 * rho * s.set.cd_kcu * norm(v_app_perp_kcu) * kcu_area) * v_app_perp_kcu
        @inbounds s.forces[spring.p2] .+= kcu_drag_force
    end

    @inbounds s.forces[spring.p1] .+= half_drag_force + s.spring_force
    @inbounds s.forces[spring.p2] .+= half_drag_force - s.spring_force
    if i == 1 s.last_force .= s.forces[spring.p1] end
    nothing
end

"""
    calc_aero_forces!(s::KPS4, pos, vel, rho, alpha_depower, rel_steering)

Calculates the aerodynamic forces acting on the kite particles.

Parameters:
- pos:              vector of the particle positions
- vel:              vector of the particle velocities
- rho:              air density [kg/m^3]
- rel_depower:      value between  0.0 and  1.0
- alpha_depower:    depower angle [degrees]
- rel_steering:     value between -1.0 and +1.0

Updates the vector s.forces of the first parameter.
"""
@inline function calc_aero_forces!(s::KPS4, pos, vel, rho, alpha_depower, rel_steering)
    rel_side_area = s.set.rel_side_area/100.0    # defined in percent
    K = 1 - rel_side_area                        # correction factor for the drag
    # pos_B, pos_C, pos_D: position of the kite particles B, C, and D
    # v_B,   v_C,   v_D:   velocity of the kite particles B, C, and D
    pos_B, pos_C, pos_D = pos[s.set.segments+3], pos[s.set.segments+4], pos[s.set.segments+5]
    v_A, v_B, v_C, v_D  = vel[s.set.segments+2], vel[s.set.segments+3], vel[s.set.segments+4], vel[s.set.segments+5]
    va_1, va_2,  va_3,  va_4  = s.v_wind - v_A, s.v_wind - v_B, s.v_wind - v_C, s.v_wind - v_D
 
    pos_centre = 0.5 * (pos_C + pos_D)
    delta = pos_B - pos_centre
    z = -normalize(delta)
    y = normalize(pos_C - pos_D)
    x = y × z
    s.x .= x; s.y .= y; s.z .= z # save the kite reference frame in the state

    va_xz1 = va_1 - (va_1 ⋅ y) * y
    va_xz2 = va_2 - (va_2 ⋅ y) * y
    va_xy3 = va_3 - (va_3 ⋅ z) * z
    va_xy4 = va_4 - (va_4 ⋅ z) * z

    R_k_w = SMatrix{3,3}([x y z])
    va_k = R_k_w' * SVector{3}(va_2)
    s.side_slip = atan(va_k[2], -va_k[1])

    alpha_2 = rad2deg(π - acos2(normalize(va_xz2) ⋅ x) - alpha_depower)     + s.set.alpha_zero
    alpha_3 = rad2deg(π - acos2(normalize(va_xy3) ⋅ x) + rel_steering * s.ks) + s.set.alpha_ztip
    alpha_4 = rad2deg(π - acos2(normalize(va_xy4) ⋅ x) - rel_steering * s.ks) + s.set.alpha_ztip
    s.alpha_2 = alpha_2
    s.alpha_3 = alpha_3
    s.alpha_4 = alpha_4

    if s.set.version == 3
        drag_corr = 1.0
    else
        drag_corr = DRAG_CORR
    end

    CL2, CD2 = s.calc_cl(alpha_2), drag_corr * s.calc_cd(alpha_2)
    CL3, CD3 = s.calc_cl(alpha_3), drag_corr * s.calc_cd(alpha_3)
    CL4, CD4 = s.calc_cl(alpha_4), drag_corr * s.calc_cd(alpha_4)
    s.side_cl = CL4 - CL3
    L2 = (-0.5 * rho * (norm(va_xz2))^2 * s.set.area * CL2) * normalize(va_2 × y)
    L3 = (-0.5 * rho * (norm(va_xy3))^2 * s.set.area * rel_side_area * CL3) * normalize(va_3 × z)
    L4 = (-0.5 * rho * (norm(va_xy4))^2 * s.set.area * rel_side_area * CL4) * normalize(z × va_4)
    D2 = (-0.5 * K * rho * norm(va_2) * s.set.area * CD2) * va_2
    D3 = (-0.5 * K * rho * norm(va_3) * s.set.area * rel_side_area * CD3) * va_3
    D4 = (-0.5 * K * rho * norm(va_4) * s.set.area * rel_side_area * CD4) * va_4
    s.lift_force .= L2
    if s.set.version == 3
        s.drag_force .= D2
        s.forces[s.set.segments + 3] .+= (L2 + D2-D3-D4)
    else
        s.drag_force .= D2 + D3 + D4
        s.forces[s.set.segments + 3] .+= (L2 + D2)
    end
    s.f_d .= (0.5 * rho * s.set.area * norm(va_xz1)^2 * s.set.cmq * s.pitch_rate * s.set.cord_length) * s.z
    s.f_s .= (0.5 * rho * s.set.area * (0.5*(norm(va_xy3)+norm(va_xy4)))^2 * s.set.smc * rel_steering * s.ks) * s.x
    s.forces[s.set.segments + 2] .+= s.f_d
    s.forces[s.set.segments + 4] .+= (L3 + D3 -0.5*s.f_d + 0.5*s.f_s)
    s.forces[s.set.segments + 5] .+= (L4 + D4 -0.5*s.f_d - 0.5*s.f_s)
    s.side_force .= (L3 + L4)
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
    damping  = s.set.damping / L_0
    c_spring = s.set.c_spring/L_0 
    for i in 1:s.set.segments
        @inbounds s.masses[i] = m_tether_particle
        @inbounds s.springs[i] = SP(s.springs[i].p1, s.springs[i].p2, s.segment_length, c_spring, damping)
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
function residual!(res, yd, y::MVector{S, SimFloat}, s::KPS4, time) where S
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
    use_brake = s.wm isa AsyncMachine
    if !isnothing(s.sync_speed) && s.sync_speed == 0.0
        use_brake = true
    end
    if s.wm isa AsyncMachine
        res[end] = v_reel_outd - calc_acceleration(s.wm, s.sync_speed, v_reel_out, norm(s.forces[1]), true)       
    else
        res[end] = v_reel_outd - calc_acceleration(s.wm, v_reel_out, norm(s.forces[1]); set_speed=s.sync_speed, 
            set_torque=s.set_torque, use_brake)
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
    cl_cd(s::KPS4)

Calculate the lift and drag coefficients of the kite, based on the current angles of attack.
"""
function cl_cd(s::KPS4)
    rel_side_area = s.set.rel_side_area/100.0  # defined in percent
    K = 1 - rel_side_area                      # correction factor for the drag
    if s.set.version == 3
        drag_corr = 1.0
    else
        drag_corr = DRAG_CORR
    end
    CL2, CD2 = s.calc_cl(s.alpha_2), drag_corr * s.calc_cd(s.alpha_2)
    CL3, CD3 = s.calc_cl(s.alpha_3), drag_corr * s.calc_cd(s.alpha_3)
    CL4, CD4 = s.calc_cl(s.alpha_4), drag_corr * s.calc_cd(s.alpha_4)
    if s.set.version == 3
        return CL2, CD2
    else
        return CL2, K*(CD2+CD3+CD4)
    end
end

# ==================== end of getter functions ================================================

function spring_forces(s::KPS4; prn=true)
    forces = zeros(SimFloat, s.set.segments+KITE_SPRINGS)
    for i in 1:s.set.segments
        forces[i] =  s.springs[i].c_spring * (norm(s.pos[i+1] - s.pos[i]) - s.segment_length) * s.stiffness_factor
        if forces[i] > s.set.max_force && prn
            println("Tether raptures for segment $i !")
        end
    end
    for i in 1:KITE_SPRINGS
        p1 = s.springs[i+s.set.segments].p1  # First point nr.
        p2 = s.springs[i+s.set.segments].p2  # Second point nr.
        pos1, pos2 = s.pos[p1], s.pos[p2]
        spring = s.springs[i+s.set.segments]
        l_0 = spring.length # Unstressed length
        k = spring.c_spring * s.stiffness_factor       # Spring constant 
        segment = pos1 - pos2
        norm1 = norm(segment)
        k1 = 0.25 * k # compression stiffness kite segments
        if (norm1 - l_0) > 0.0
            spring_force = k *  (norm1 - l_0) 
        else 
            spring_force = k1 *  (norm1 - l_0)
        end
        forces[i+s.set.segments] = spring_force
        if norm(s.spring_force) > 4000.0
            println("Bridle brakes for spring $i !")
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
    find_steady_state!(s::KPS4; prn=false, delta = 0.01, stiffness_factor=0.035, upwind_dir=-pi/2))

Find an initial equilibrium, based on the initial parameters
`l_tether`, elevation and `v_reel_out`.
"""
function find_steady_state!(s::KPS4; prn=false, delta = 0.01, stiffness_factor=0.035, upwind_dir=-pi/2)
    set_v_wind_ground!(s, calc_height(s), s.set.v_wind; upwind_dir=-pi/2)
    s.stiffness_factor = stiffness_factor
    res = zeros(MVector{6*(s.set.segments+KITE_PARTICLES)+2, SimFloat})
    iter = 0

    # helper function for the steady state finder
    function test_initial_condition!(F, x::Vector)
        x1 = copy(x)
        y0, yd0 = init(s, x1; delta)
        try
            residual!(res, yd0, y0, s, 0.0)
        catch e
            println("Warning in test_initial_condition!")
        end
        for i in 1:s.set.segments+KITE_PARTICLES-1
            if i != s.set.segments+KITE_PARTICLES-1
                j = i
            else
                j = i + 1
            end
            # copy the x-component of the residual res2 (acceleration)
            F[i]                               = res[1 + 3*(j-1) + 3*(s.set.segments+KITE_PARTICLES)]
            # copy the z-component of the residual res2
            F[i+s.set.segments+KITE_PARTICLES] = res[3 + 3*(j-1) + 3*(s.set.segments+KITE_PARTICLES)]
        end
        # copy the acceleration of point KCU in x direction
        i = s.set.segments+1
        F[end-1]                               = res[1 + 3*(i-1) + 3*(s.set.segments+KITE_PARTICLES)] 
        # copy the acceleration of point C in y direction
        i = s.set.segments+3 
        x = res[1 + 3*(i-1) + 3*(s.set.segments+KITE_PARTICLES)]
        y = res[2 + 3*(i-1) + 3*(s.set.segments+KITE_PARTICLES)]
        F[end]                                 = res[2 + 3*(i-1) + 3*(s.set.segments+KITE_PARTICLES)] 
        iter += 1
        return nothing 
    end
    if prn println("\nStarted function test_nlsolve...") end
    X00 = zeros(SimFloat, 2*(s.set.segments+KITE_PARTICLES-1)+2)
    results = nlsolve(test_initial_condition!, X00, autoscale=true, xtol=4e-7, ftol=4e-7, iterations=s.set.max_iter)
    if prn println("\nresult: $results") end
    y0, yd0 = init(s, results.zero; upwind_dir)
    set_v_wind_ground!(s, calc_height(s), s.set.v_wind; upwind_dir)
    residual!(res, yd0, y0, s, 0.0)
    y0, yd0
end
