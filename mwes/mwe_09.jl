# SPDX-FileCopyrightText: 2025 Uwe Fechner
#
# SPDX-License-Identifier: MIT

using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
if ! ("OrdinaryDiffEq" ∈ keys(Pkg.project().dependencies))
    Pkg.add("OrdinaryDiffEq")
end
using OrdinaryDiffEq, ControlPlots, StaticArrays

STATIC_ARRAYS::Bool = false
if STATIC_ARRAYS
    const G_EARTH  = SA[0.0, 0.0, -9.81] # gravitational acceleration
else
    const G_EARTH  = [0.0, 0.0, -9.81] # gravitational acceleration
end
const dt = 0.05
const t_final = 10.0
  
# Example one: Falling mass
# State vector y   = mass.pos, mass.vel
# Derivative   yd  = mass.vel, mass.acc
# Residual     res = (y.vel - yd.vel), (yd.acc - G_EARTH)     
function res!(res, yd, y, p, time)
    @views res[1:3] .= y[4:6] .- yd[1:3]
    @views res[4:6] .= yd[4:6] .- G_EARTH
    nothing
end

struct Result
    time::Vector{Float64}
    pos_z::Vector{Float64}
    vel_z::Vector{Float64}
end
function Result(t_final)
    n=Int64(round(t_final/dt+1))
    Result(zeros(n), zeros(n), zeros(n))
end

function solve!(res, integrator, dt, t_final)
    for (i,t) in pairs(0:dt:t_final)
        res.time[i] = t
        step!(integrator, dt, true)
        res.pos_z[i] = integrator.u[3]
        res.vel_z[i] = integrator.u[6]
    end
    nothing
end

function init()
    vel_0 = [0.0, 0.0, 50.0]    # Initial velocity
    pos_0 = [0.0, 0.0,  0.0]    # Initial position
    acc_0 = [0.0, 0.0, -9.81]   # Initial acceleration
    y0 = append!(pos_0, vel_0)  # Initial pos, vel
    yd0 = append!(vel_0, acc_0) # Initial vel, acc
    if STATIC_ARRAYS
        y0 = MVector{6}(y0)
        yd0 = MVector{6}(yd0)
    end
    println(typeof(y0))

    # solver  = IDA(linear_solver=:GMRES, max_order = 4) # 525 bytes, 0.228 ms
    # solver = DImplicitEuler(autodiff=false)            # 321 bytes, 0.341 ms
    solver = DFBDF(autodiff=false, max_order=Val{4}())   #  87 bytes, 0.261 ms
    # solver = DABDF2(autodiff=false)                    #   1 bytes, 0.319 ms
    
    tspan   = (0.0, t_final) 
    abstol  = 0.0006 # max error in m/s and m
    reltol=0.001 #* ones(length(y0))
    s = nothing

    prob    = DAEProblem(res!, yd0, y0, tspan, s)
    integrator = OrdinaryDiffEq.init(prob, solver; abstol, reltol, save_everystep=false)
end

function plot(res::Result)
    plt.ax1 = plt.subplot(111) 
    plt.ax1.set_xlabel("time [s]")
    plt.plot(res.time, res.pos_z, color="green")
    plt.ax1.set_ylabel("pos_z [m]")  
    plt.ax1.grid(true) 
    plt.ax2 = plt.twinx()  
    plt.ax2.set_ylabel("vel_z [m/s]")   
    plt.plot(res.time, res.vel_z, color="red")
end

integrator=init()
res=Result(t_final)
@time solve!(res, integrator, dt, t_final)
integrator=init()
res=Result(t_final)
bytes = @allocated solve!(res, integrator, dt, t_final)
n=Int64(round(t_final/dt+1))
println("Allocated $(Int64(round(bytes/n))) bytes per iteration!")
integrator=init()
res=Result(t_final)
@timev solve!(res, integrator, dt, t_final)
# plot(res)
# Allocated   87 bytes per iteration! (STATIC_ARRAYS=false)
# Allocated 3052 bytes per iteration! (STATIC_ARRAYS=true)