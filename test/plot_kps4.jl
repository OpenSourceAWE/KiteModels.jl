# SPDX-FileCopyrightText: 2022, 2024, 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

# activate the test environment if needed
using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end

using KiteModels
using KitePodModels
using KiteUtils
using MakieControlPlots
using LinearAlgebra

kcu::KCU = KCU(se())
kps4::KPS4 = KPS4(kcu)

const FRONT_VIEW = false

clear!(kps4)
kps4.stiffness_factor = 0.04

y0, yd0 = KiteModels.init(kps4)

find_steady_state!(kps4; delta=0.001, stiffness_factor=1, prn=false)

println("kite distance: $(norm(kps4.pos[end-2]))")
println(KiteModels.spring_forces(kps4))
println("alpha_depower [deg]: $(rad2deg(kps4.alpha_depower))")
println("lift, drag    [N]  : $(KiteModels.lift_drag(kps4))")

plot2d(kps4.pos; zoom=false, front=FRONT_VIEW)
nothing