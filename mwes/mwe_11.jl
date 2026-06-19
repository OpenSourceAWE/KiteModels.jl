# SPDX-FileCopyrightText: 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
Pkg.activate(joinpath(@__DIR__,"..","examples"))
Pkg.instantiate()
using MakieControlPlots
using KiteModels, KitePodModels, KiteUtils

set = deepcopy(se())
kcu::KCU = KCU(set)
kps3::KPS3 = KPS3(kcu)


reltime = 0.0
integrator = KiteModels.init!(kps3, stiffness_factor=0.04)
plot2d(kps3.pos, reltime; zoom=false, front=false, segments=set.segments)