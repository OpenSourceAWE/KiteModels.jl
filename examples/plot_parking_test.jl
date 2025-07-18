# Copyright (c) 2022, 2024 Uwe Fechner
# SPDX-License-Identifier: MIT

# plot the lift and drag coefficients as function of angle of attack

# t_start  t_stop   duration  depower  height   av_elevation std_elevation av_pitch  av_wind_200 
# ────────────────────────────────────────────────────────────────────────────────────────────────
# 11624.9  11653.3      28.4    40.0     268.3       70.6789       1.26185    12.8284      10.23
# 11538.2  11559.5      21.3    44.0     250.1       65.2369       0.821425   13.7453      11.53
# 12866.4  12886.6      20.2    51.98    249.4       57.9619       0.92951    14.0217      12.0867
# 11472.8  11490.6      17.8    47.99    237.5       61.4089       0.648852   14.8983      11.92


ELEV_MEASURED = [70.6789, 65.2369, 61.4089, 57.9619]
STD_ELEVATION = [1.26185, 0.821425, 0.648852, 0.92951]
PITCH         = [12.8284, 13.7453, 14.8983, 14.0217]
V_WIND_200    = [ 10.23, 11.53, 12.0867, 11.92]
HEIGHT        = [ 268.3, 250.1, 249.4, 237.5]
DEPOWER       = [0.40, 0.44, 0.4799, 0.5198]

using Printf
using KiteModels, LinearAlgebra

if haskey(ENV, "USE_V9")
    set = deepcopy(load_settings("system_v9.yaml"))
else
    set = deepcopy(load_settings("system.yaml"))
end

using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using ControlPlots
plt.close("all")

set.abs_tol=0.0006
set.rel_tol=0.00001

# the following values can be changed to match your interest
dt = 0.05
set.solver="DFBDF" # IDA or DFBDF
STEPS = 550# 740
PLOT = true
PRINT = true
STATISTIC = false
# end of user parameter section #

bridle_length = KiteModels.bridle_length(set)
println("bridle_length: $bridle_length")
bridle_area = (set.d_line/2000) * bridle_length
println("bridle_area: $bridle_area")

function set_tether_diameter!(se, d; c_spring_4mm = 614600, damping_4mm = 473)
    set.d_tether = d
    set.c_spring = c_spring_4mm * (d/4.0)^2
    set.damping = damping_4mm * (d/4.0)^2
end

set_tether_diameter!(set, set.d_tether)

if PLOT
    using Pkg
    if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
        using TestEnv; TestEnv.activate()
    end
    using ControlPlots
end

function simulate(kps4, integrator, logger, steps)
    iter = 0
    cl = 0.0
    cd = 0.0
    for i in 1:steps
        force = norm(kps4.forces[1])
        r = set.drum_radius
        n = set.gear_ratio
        set_torque = -r/n * force
        next_step!(kps4, integrator; set_torque, dt)
        sys_state = KiteModels.SysState(kps4)
        aoa = kps4.alpha_2
        sys_state.var_01 = aoa
        log!(logger, sys_state)
        iter += kps4.iter
        if i > steps - 50 # last 2.5s
            cl_, cd_ = KiteModels.cl_cd(kps4)
            cl += cl_
            cd += cd_
        end
    end
    return cl/50, cd/50
end


CL = zeros(length(DEPOWER))
CD = zeros(length(DEPOWER))
AOA = zeros(length(DEPOWER))
DEP = zeros(length(DEPOWER))
ELEV = zeros(length(DEPOWER))
V_WIND_KITE = zeros(length(DEPOWER))

elev = set.elevation
i = 1
for depower in DEPOWER
    global elev, i, kps4
    local cl, cd, aoa, kcu, integrator, logger, v_app

    logger = Logger(set.segments + 5, STEPS)
    DEP[i] = depower
    set.depower = 100*depower
    
    # set.depower_gain = 5

    kcu::KCU = KCU(set)
    kps4::KPS4 = KPS4(kcu)
    set.elevation += 5
    set.v_wind = V_WIND_200[i] / 1.348881340489221 * calc_wind_factor(kps4.am, HEIGHT[i])/1.348881340489221
    integrator = KiteModels.init!(kps4; delta=0.001*0, stiffness_factor=1, prn=STATISTIC)
    if ! isnothing(integrator)
        try
            cl, cd = simulate(kps4, integrator, logger, STEPS)
        catch e
            println("Error: $e")
            if PLOT
                p = plot(logger.time_vec, rad2deg.(logger.elevation_vec), logger.var_01_vec, xlabel="time [s]", ylabels=["elevation [°]", "aoa"], 
                         fig="depower: $depower")
                display(p)
                sleep(0.2)
            end        
            break
        end
        set_depower_steering(kps4.kcu, depower, 0.0)
        cl, cd = simulate(kps4, integrator, logger, STEPS)
    else
        break
    end
    elev = rad2deg(logger.elevation_vec[end])
    ELEV[i] = elev
    V_WIND_KITE[i] = norm(kps4.v_wind)
    set.elevation -= 5
    if elev > 70
        set.elevation = elev - 4
    else
        set.elevation = elev - 4
    end 
    j=i
    if j ==2
        set.elevation -= 4
    elseif j == 3
        set.elevation -= 4
    elseif j == 4
        set.elevation += 4
    end

    aoa = kps4.alpha_2
    orient_vec = orient_euler(kps4)
    alpha_depower = rad2deg(kps4.alpha_depower)
    pitch = rad2deg(orient_vec[2]) + alpha_depower
    v_app = norm(kps4.v_apparent)
    # v_200 = calc_wind_factor(kps4.am, 200) * V_WIND
    height = logger.Z_vec[end][end-2]
    CL[i] = cl
    CD[i] = cd
    AOA[i] = aoa
    if PRINT
        print("Depower: $depower, alpha_dp: $(round(alpha_depower, digits=2)), CL $(round(cl, digits=3)), CD: $(round(cd, digits=3)), aoa: $(round(aoa, digits=2)), pitch: $(round(pitch, digits=2)), CL/CD: $(round(cl/cd, digits=2))")
        println(", elevation: $(round((elev), digits=2)), height:$(round(height, digits=2))")
    end
    # if depower in [DEPOWER[begin+1], DEPOWER[end]] && PLOT
    if PLOT
        p = plot(logger.time_vec, rad2deg.(logger.elevation_vec), logger.var_01_vec, xlabel="time [s]", ylabels=["elevation [°]", "aoa [°]"], 
                 fig="depower: $depower")
        display(p)
        sleep(0.2)
    end
    i+=1
end

cl = zeros(length(AOA))
cd = zeros(length(AOA))
for (i, alpha) in pairs(AOA)
    global cl, cd
    cl[i] = kps4.calc_cl(alpha)
    cd[i] = kps4.calc_cd(alpha)
end

# display(plot(AOA, [CL, cl], xlabel="AOA [deg]", ylabel="CL", labels=["CL","cl"], fig="CL vs AOA"))
# display(plot(AOA, [CD, cd], xlabel="AOA [deg]", ylabel="CD", labels=["CD","cd"], fig="CD vs AOA"))
display(plot(DEP, AOA, xlabel="Depower", ylabel="AOA [deg]", fig="AOA vs Depower"))
display(plot(DEP,[(ELEV, nothing), (ELEV_MEASURED, 2*STD_ELEVATION)]; 
        xlabel="depower", ylabel="elevation [°]", scatter=true, 
        labels=["simulated", "measured ±2σ"], fig="elevation vs depower"))
# display(plot(DEP,[AOA, 1.5*25.5 .- 2PITCH]; xlabel="depower", ylabel="aoa/pitch [°]", scatter=true, labels=["aoa", "38.25°-2pitch"], fig="pitch and aoa vs depower"))
