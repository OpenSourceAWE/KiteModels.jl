# SPDX-FileCopyrightText: 2025 Uwe Fechner
# SPDX-License-Identifier: MIT

using Pkg
if ! ("Test" ∈ keys(Pkg.project().dependencies))
    Pkg.activate("test")
end

using KiteUtils, LinearAlgebra, StaticArrays
using KiteModels, KitePodModels

set_data_path(joinpath(dirname(dirname(pathof(KiteModels))), "data"))
set = deepcopy(load_settings("system.yaml"))
kcu::KCU = KCU(set)

kps4::KPS4 = KPS4(kcu)

dt = 0.05

clear!(kps4)
KiteModels.set_depower_steering!(kps4, kps4.set.depower_offset/100.0, 0.0)
kps4.stiffness_factor = 0.5

KiteModels.find_steady_state!(kps4, prn=false)

# println("\nSpring forces:")
# spring_forces(kps4)
