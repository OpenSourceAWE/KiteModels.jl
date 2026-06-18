# SPDX-FileCopyrightText: 2025 Uwe Fechner
#
# SPDX-License-Identifier: MIT

using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using MakieControlPlots
using KiteModels, KitePodModels, KiteUtils

set = deepcopy(se())
kcu::KCU = KCU(set)
kps3::KPS3 = KPS3(kcu)


reltime = 0.0
integrator = KiteModels.init!(kps3, stiffness_factor=0.04)
plot2d(kps3.pos, reltime; zoom=false, front=false, segments=set.segments)